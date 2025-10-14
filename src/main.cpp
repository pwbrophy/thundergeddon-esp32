// ============================= INCLUDES =============================		// Section header
#include <Arduino.h>
#include "secrets.h"		// Wi-Fi credentials
#include <esp32-hal-ledc.h>
#include "esp_camera.h"		// Camera driver
#include "soc/soc.h"		// SoC registers (legacy include kept)
#include "soc/rtc_cntl_reg.h"		// Brownout/RTC control regs (legacy include kept)
#include <WiFi.h>		// Wi-Fi API
#include <WiFiClientSecure.h>		// TLS client type required by ArduinoWebsockets
#include <Wire.h>		// I2C API
#include <ArduinoWebsockets.h>		// WebSocket client
#include <Adafruit_PWMServoDriver.h>		// PCA9685 driver
#include <math.h>		// Math helpers for motor scaling
#include <SPI.h>    // SPI API (not used, but needed to link with Adafruit lib)

// ========================== CAMERA PROFILE ==========================		// Section header
#define CAMERA_MODEL_S3_WROOM		// Build profile macro used by your camera_pins.h
#include "camera_pins.h"		// Camera pin mapping
#include "secrets.h"		// Wi-Fi credentials

// ============================== CONFIG ==============================		// Section header

const char* role = "robot_tank";

const char* websockets_server = "ws://192.168.86.197:8080/esp32";		// WebSocket server URL

#define DBG 1		// Debug flag (1=enable Serial logs, 0=disable)

constexpr int I2C_SDA_PIN = 21;		// I2C SDA pin
constexpr int I2C_SCL_PIN = 47;		// I2C SCL pin
constexpr uint8_t PCA_ADDR = 0x40;		// PCA9685 I2C address
constexpr uint8_t OE_PIN = 41;		// PCA9685 OE pin (active-LOW, driven by LEDC)
constexpr uint16_t PWM_FREQ = 2000;		// PCA9685 PWM frequency for motors (Hz)
constexpr uint8_t SLEEP_CH = 14;		// PCA9685 channel for DRV8833 nSLEEP (active-HIGH)

constexpr uint8_t L_IN1_CH = 10;		// Left motor A channel
constexpr uint8_t L_IN2_CH = 11;		// Left motor B channel
constexpr uint8_t R_IN1_CH = 0;		// Right motor A channel
constexpr uint8_t R_IN2_CH = 1;		// Right motor B channel
constexpr uint8_t T_IN1_CH = 8;		// Turret A channel
constexpr uint8_t T_IN2_CH = 9;		// Turret B channel

constexpr uint8_t IR_LED_CH_N = 2;		// IR LED North channel
constexpr uint8_t IR_LED_CH_NE = 3;		// IR LED North-East channel
constexpr uint8_t IR_LED_CH_E = 4;		// IR LED East channel
constexpr uint8_t IR_LED_CH_SE = 5;		// IR LED South-East channel
constexpr uint8_t IR_LED_CH_S = 6;		// IR LED South channel
constexpr uint8_t IR_LED_CH_SW = 7;		// IR LED South-West channel
constexpr uint8_t IR_LED_CH_W = 13;		// IR LED West channel
constexpr uint8_t IR_LED_CH_NW = 12;		// IR LED North-West channel

constexpr int IR_PIN = 39;		// TSOP receiver GPIO (use INPUT_PULLUP)
constexpr int BUZZ_PIN = 1;		// Buzzer GPIO (LEDC tone output)
constexpr int BEEP_HZ = 1000;		// Buzzer tone frequency (Hz)
constexpr int BUZZ_CH_BITS = 10;		// Buzzer PWM resolution (bits)

const int frameRate = 30;		// Target camera FPS
const int frameDelay = 1000 / frameRate;		// Frame interval in ms
static bool g_cameraOk = false;		// Camera init status

constexpr int IR_LEDC_CH = 7;   // 0..7
constexpr int BUZZ_CH    = 6;   // 0..7

Adafruit_PWMServoDriver pca(PCA_ADDR);		// PCA9685 instance
using namespace websockets;		// Use websockets namespace
WebsocketsClient client;		// WebSocket client instance

// ============================== LOGGING ==============================		// Section header
#if DBG		// Debug macro block begin
  #define LOGF(...) do{ Serial.printf(__VA_ARGS__); }while(0)		// Formatted log helper
  #define LOG(msg)  do{ Serial.println(msg); }while(0)		// Simple log helper
#else		// Debug macro block else
  #define LOGF(...)		// No-op when DBG==0
  #define LOG(msg)		// No-op when DBG==0
#endif		// Debug macro block end

// ============================== HELPERS ==============================		// Section header
inline void pinHigh(uint8_t ch){ pca.setPWM(ch, 4096, 0); }		// Drive PCA channel fully HIGH (brake)
inline void pinLow(uint8_t ch){ pca.setPWM(ch, 0, 4096); }		// Drive PCA channel fully LOW (coast)
inline void pinPWM_inverted(uint8_t ch, uint16_t d){		// Set inverted PWM on PCA channel
  if(d == 0) pinHigh(ch);		// If duty==0 → drive HIGH (off for low-side)
  else if(d >= 4095) pinLow(ch);		// If duty max → drive LOW (full on for low-side)
  else pca.setPWM(ch, d, 0);		// Otherwise set inverted duty
}		// End pinPWM_inverted

void setMotorSpeed(uint8_t chA, uint8_t chB, float s){		// Bidirectional motor speed helper
  s = fmaxf(-1.f, fminf(1.f, s));		// Clamp to [-1,1]
  const float dead = 0.02f;		// Deadband threshold
  if(fabsf(s) < dead){ pinHigh(chA); pinHigh(chB); return; }		// Brake when within deadband
  uint16_t duty = (uint16_t)roundf(fabsf(s) * 4095.f);		// Map magnitude to 0..4095 duty
  if(s > 0){ pinHigh(chA); pinPWM_inverted(chB, duty); }		// Forward: A=HIGH, PWM on B
  else { pinHigh(chB); pinPWM_inverted(chA, duty); }		// Reverse: B=HIGH, PWM on A
}		// End setMotorSpeed

String deviceId(){		// Unique ID from MAC
  uint64_t mac = ESP.getEfuseMac();		// Read 48-bit MAC
  char id[13];		// Hex string buffer (12 chars + NUL)
  snprintf(id, sizeof(id), "%02X%02X%02X%02X%02X%02X",
           (uint8_t)(mac >> 40),
           (uint8_t)(mac >> 32),
           (uint8_t)(mac >> 24),
           (uint8_t)(mac >> 16),
           (uint8_t)(mac >> 8),
           (uint8_t)mac);		// Format MAC to hex
  return String(id);		// Return as Arduino String
}		// End deviceId

void printRingMap(){		// Human-readable IR LED direction map
  LOG("[MAP] Direction -> PCA channel");		// Print header
  LOGF("[MAP] N  -> %u\n", IR_LED_CH_N);		// Map N
  LOGF("[MAP] NE -> %u\n", IR_LED_CH_NE);		// Map NE
  LOGF("[MAP] E  -> %u\n", IR_LED_CH_E);		// Map E
  LOGF("[MAP] SE -> %u\n", IR_LED_CH_SE);		// Map SE
  LOGF("[MAP] S  -> %u\n", IR_LED_CH_S);		// Map S
  LOGF("[MAP] SW -> %u\n", IR_LED_CH_SW);		// Map SW
  LOGF("[MAP] W  -> %u\n", IR_LED_CH_W);		// Map W
  LOGF("[MAP] NW -> %u\n", IR_LED_CH_NW);		// Map NW
}		// End printRingMap

// =========================== EMITTER (TX) ===========================		// Section header
constexpr uint32_t FIRE_US = 1000;		// Pulse width in microseconds
constexpr double IR_CARRIER_HZ = 38000.0;		// Carrier frequency in Hz
constexpr uint8_t IR_DUTY_BITS = 8;		// PWM resolution bits
constexpr uint32_t IR_DUTY_50 = 128;		// ~50% duty at 8 bits

static bool carrierOn = false;		// Carrier running flag
static bool g_emit = false;		// Emit in progress flag
static bool g_emitReported = false;		// Emit completion reported flag
static uint32_t g_emitEndUs = 0;		// Emit end timestamp (us)
static uint8_t g_emitCh = IR_LED_CH_N;		// Current IR LED channel

inline void irOn(uint8_t ch){ pca.setPin(ch, 4095, false); }		// Turn specific IR LED ON (sink current)
inline void irOff(uint8_t ch){ pca.setPin(ch, 0, false); }		// Turn specific IR LED OFF

void carrierStart() {
  if (carrierOn) return;
  ledcSetup(IR_LEDC_CH, IR_CARRIER_HZ, IR_DUTY_BITS);
  ledcAttachPin(OE_PIN, IR_LEDC_CH);
  ledcWrite(IR_LEDC_CH, IR_DUTY_50);
  carrierOn = true;
}

void carrierStop() {
  if (!carrierOn) return;
  ledcWrite(IR_LEDC_CH, 0);
  ledcDetachPin(OE_PIN);
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, LOW);
  carrierOn = false;
}

uint8_t dirToCh(const String& d){		// Map direction string to PCA channel
  if(d == "N") return IR_LED_CH_N;		// N
  if(d == "NE") return IR_LED_CH_NE;		// NE
  if(d == "E") return IR_LED_CH_E;		// E
  if(d == "SE") return IR_LED_CH_SE;		// SE
  if(d == "S") return IR_LED_CH_S;		// S
  if(d == "SW") return IR_LED_CH_SW;		// SW
  if(d == "W") return IR_LED_CH_W;		// W
  return IR_LED_CH_NW;		// Default: NW
}		// End dirToCh

void emitStart(uint8_t ch, const char* dirStr){		// Begin 1 ms IR burst on channel
  if(g_emit) return;		// Ignore if already emitting
  g_emit = true;		// Mark emitting
  g_emitReported = false;		// Clear reported flag
  g_emitCh = ch;		// Set channel
  g_emitEndUs = micros() + FIRE_US;		// Compute end time
  if(!carrierOn){ pinMode(OE_PIN, OUTPUT); digitalWrite(OE_PIN, LOW); carrierStart(); }		// Ensure carrier
  irOn(g_emitCh);		// Turn IR LED ON
  LOGF("[EMIT] dir=%s ch=%u t0_us=%lu\n", dirStr, (unsigned)g_emitCh, (unsigned long)micros());		// Log start
}		// End emitStart

void emitTick(){		// Maintain emit timing and report completion
  if(!g_emit) return;		// Only if emitting
  if((int32_t)(micros() - g_emitEndUs) >= 0){		// Time elapsed?
    irOff(g_emitCh);		// Turn IR LED OFF
    g_emit = false;		// Clear emitting flag
    if(!g_emitReported){ client.send("{\"cmd\":\"emit_done\"}"); g_emitReported = true; LOG("[EMIT] done"); }		// Notify host
  }		// End if elapsed
}		// End emitTick

// =========================== RECEIVER (RX) ===========================		// Section header
static volatile uint32_t v_pulsesSinceClear = 0;		// Pulse counter since last mark
static bool listenActive = false;		// Listening state flag
static bool results[8];		// Hit results per direction

int dirToIdx(const String& d){		// Map direction string to index 0..7
  if(d == "N") return 0;		// N
  if(d == "NE") return 1;		// NE
  if(d == "E") return 2;		// E
  if(d == "SE") return 3;		// SE
  if(d == "S") return 4;		// S
  if(d == "SW") return 5;		// SW
  if(d == "W") return 6;		// W
  return 7;		// NW
}		// End dirToIdx

const char* idxToDir(int i){ static const char* d[8] = {"N","NE","E","SE","S","SW","W","NW"}; return d[(i & 7)]; }		// Map index to direction string

void IRAM_ATTR isrIR(){		// ISR for TSOP falling edges
  if(!listenActive) return;		// Ignore if not armed
  v_pulsesSinceClear++;		// Count one pulse
}		// End isrIR

void listenArmAndClear(){		// Arm receiver and clear state
  for(int i = 0; i < 8; i++) results[i] = false;		// Clear results
  listenActive = true;		// Enable ISR counting
  noInterrupts();		// Enter critical section
  v_pulsesSinceClear = 0;		// Reset pulse counter
  interrupts();		// Exit critical section
  setMotorSpeed(L_IN1_CH, L_IN2_CH, 0);		// Stop left motor
  setMotorSpeed(R_IN1_CH, R_IN2_CH, 0);		// Stop right motor
  setMotorSpeed(T_IN1_CH, T_IN2_CH, 0);		// Stop turret
  client.send("{\"cmd\":\"ready\"}");		// Notify host ready
  LOG("[RX] armed + motors off");		// Log state
}		// End listenArmAndClear

void listenFinishAndSendResults(){		// Stop listening and send results
  listenActive = false;		// Disable ISR counting
  String json = "{\"cmd\":\"scan_results\",\"hits\":[";		// Begin JSON payload
  bool first = true;		// Comma control
  for(int i = 0; i < 8; i++){		// Iterate directions
    if(results[i]){ if(!first) json += ","; json += "\""; json += idxToDir(i); json += "\""; first = false; }		// Append hit
  }		// End loop
  json += "]}";		// Close JSON payload
  client.send(json);		// Send to host
  LOG("[RX] finished; results sent");		// Log state
}		// End listenFinishAndSendResults

void markDirection(const String& dir){		// Commit pulses to a direction
  int idx = dirToIdx(dir);		// Map to index
  uint32_t pulses;		// Local pulse copy
  noInterrupts();		// Enter critical section
  pulses = v_pulsesSinceClear;		// Snapshot pulses
  v_pulsesSinceClear = 0;		// Clear for next window
  interrupts();		// Exit critical section
  bool hit = (pulses > 0);		// Determine hit
  results[idx] = results[idx] || hit;		// Accumulate idempotently
  LOGF("[RX] MARK dir=%s idx=%d pulses=%lu -> %s\n", dir.c_str(), idx, (unsigned long)pulses, hit ? "HIT" : "MISS");		// Log mark
}		// End markDirection

// =========================== JSON HELPERS ===========================		// Section header
static bool jsonHasCmd(const String& s, const char* cmd){ return s.indexOf(String("\"cmd\":\"") + cmd + "\"") >= 0; }		// Search for cmd key
static bool jsonGetFloat(const String& s, const char* key, float& out){		// Extract float by key
  int k = s.indexOf(String("\"") + key + "\"");		// Find key
  if(k < 0) return false;		// Not found
  int c = s.indexOf(':', k);		// Find colon
  if(c < 0) return false;		// Malformed
  int i = c + 1;		// Value start
  while(i < (int)s.length() && s[i] == ' ') i++;		// Skip spaces
  int j = i;		// Value end cursor
  while(j < (int)s.length()){ char ch = s[j]; if(ch == ',' || ch == '}' || isspace((unsigned char)ch)) break; j++; }		// Scan token
  if(j <= i) return false;		// Empty
  out = s.substring(i, j).toFloat();		// Convert to float
  return true;		// Success
}		// End jsonGetFloat

static bool jsonGetString(const String& s, const char* key, String& out){		// Extract string by key
  int k = s.indexOf(String("\"") + key + "\"");		// Find key
  if(k < 0) return false;		// Not found
  int c = s.indexOf(':', k);		// Find colon
  if(c < 0) return false;		// Malformed
  int q1 = s.indexOf('"', c + 1);		// First quote
  if(q1 < 0) return false;		// Missing
  int q2 = s.indexOf('"', q1 + 1);		// Second quote
  if(q2 < 0) return false;		// Missing
  out = s.substring(q1 + 1, q2);		// Extract substring
  return true;		// Success
}		// End jsonGetString

// =========================== WEBSOCKET RX ===========================		// Section header
void onMessage(websockets::WebsocketsMessage m){		// WebSocket message handler
  if(!m.isText()) return;		// Ignore non-text
  String s = m.data();		// Get payload

  if(jsonHasCmd(s, "drive")){		// Drive command
    float l = 0, r = 0;		// Local variables
    if(jsonGetFloat(s, "left", l) && jsonGetFloat(s, "right", r)){ setMotorSpeed(L_IN1_CH, L_IN2_CH, l); setMotorSpeed(R_IN1_CH, R_IN2_CH, r); }		// Set speeds
    return;		// Done
  }		// End drive

  if(jsonHasCmd(s, "turret")){		// Turret command
    float t = 0;		// Local variable
    if(jsonGetFloat(s, "speed", t)) setMotorSpeed(T_IN1_CH, T_IN2_CH, t);		// Set turret speed
    return;		// Done
  }		// End turret

  if(strcmp(ROLE, "robot_tank") == 0){		// Emitter role
    if(jsonHasCmd(s, "carrier_on")){ carrierStart(); return; }		// Start carrier
    if(jsonHasCmd(s, "carrier_off")){ carrierStop(); return; }		// Stop carrier
    if(jsonHasCmd(s, "emit")){ String d = "N"; jsonGetString(s, "dir", d); emitStart(dirToCh(d), d.c_str()); return; }		// Emit pulse
  } else {		// Receiver role
    if(jsonHasCmd(s, "listen_prepare")){ listenArmAndClear(); return; }		// Arm listening
    if(jsonHasCmd(s, "mark")){ String d = "N"; jsonGetString(s, "dir", d); markDirection(d); return; }		// Mark direction
    if(jsonHasCmd(s, "listen_finish")){ listenFinishAndSendResults(); return; }		// Send results
    if(jsonHasCmd(s, "motors_on")){ LOG("[RX] motors_on (no-op)"); return; }		// Optional hook
  }		// End role branch
}		// End onMessage

// ============================== CAMERA ==============================		// Section header
bool initCamera(){		// Initialize camera with current profile
  camera_config_t cfg = {};		// Zeroed config
  cfg.ledc_channel = LEDC_CHANNEL_0;		// LEDC channel
  cfg.ledc_timer = LEDC_TIMER_0;		// LEDC timer
  cfg.pin_d0 = Y2_GPIO_NUM;		// Data D0
  cfg.pin_d1 = Y3_GPIO_NUM;		// Data D1
  cfg.pin_d2 = Y4_GPIO_NUM;		// Data D2
  cfg.pin_d3 = Y5_GPIO_NUM;		// Data D3
  cfg.pin_d4 = Y6_GPIO_NUM;		// Data D4
  cfg.pin_d5 = Y7_GPIO_NUM;		// Data D5
  cfg.pin_d6 = Y8_GPIO_NUM;		// Data D6
  cfg.pin_d7 = Y9_GPIO_NUM;		// Data D7
  cfg.pin_xclk = XCLK_GPIO_NUM;		// XCLK pin
  cfg.pin_pclk = PCLK_GPIO_NUM;		// PCLK pin
  cfg.pin_vsync = VSYNC_GPIO_NUM;		// VSYNC pin
  cfg.pin_href = HREF_GPIO_NUM;		// HREF pin
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;   // was pin_sscb_sda
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;   // was pin_sscb_scl
  cfg.pin_pwdn = PWDN_GPIO_NUM;		// PWDN pin (may be -1)
  cfg.pin_reset = RESET_GPIO_NUM;		// RESET pin (may be -1)
  cfg.xclk_freq_hz = 22000000;		// XCLK frequency
  cfg.pixel_format = PIXFORMAT_JPEG;		// JPEG output
  cfg.frame_size = FRAMESIZE_VGA;		// Frame size
  cfg.jpeg_quality = 10;		// JPEG quality (lower = better quality)
  cfg.fb_count = 2;		// Frame buffers

  g_cameraOk = (esp_camera_init(&cfg) == ESP_OK);		// Initialize camera
  sensor_t* s = esp_camera_sensor_get();		// Get sensor handle
  if(!s) return false;		// Bail if no sensor
  s->set_hmirror(s, 1);		// Horizontal mirror on
  s->set_vflip(s, 1);		// Vertical flip on
  return g_cameraOk;		// Return status
}		// End initCamera

void sendCamera(){		// Periodically send a frame over WebSocket
  if(!g_cameraOk) return;		// Skip if camera off
  static unsigned long last = 0;		// Last send timestamp
  if(millis() - last < frameDelay) return;		// Throttle to frameRate
  last = millis();		// Update timestamp
  camera_fb_t* fb = esp_camera_fb_get();		// Grab frame
  if(!fb) return;		// Skip if grab failed
  if(!client.sendBinary((const char*)fb->buf, fb->len)) { /* no action */ }		// Send frame
  esp_camera_fb_return(fb);		// Return buffer
}		// End sendCamera

// ============================== NETWORK ==============================		// Section header
void initWifi(){		// Connect to Wi-Fi
  Serial.begin(115200);		// Start serial (if not already)
  Serial.print("[WIFI] Connecting to ");		// Log prefix
  Serial.println(WIFI_SSID);		// Log SSID
  WiFi.mode(WIFI_STA);		// Station mode
  WiFi.begin(WIFI_SSID, WIFI_PASS);		// Begin connect
  while(WiFi.status() != WL_CONNECTED){ Serial.print("."); delay(300); }		// Wait for link
  Serial.print("\n[WIFI] IP: ");		// Log prefix
  Serial.println(WiFi.localIP());		// Log IP
}		// End initWifi

void connectWS(){		// Connect WebSocket and set handlers
  client.close();		// Ensure clean state
  delay(200);		// Small gap
  client.onMessage(onMessage);		// Register RX handler
  client.onEvent([](websockets::WebsocketsEvent e, String){		// Event callback
    if(e == websockets::WebsocketsEvent::ConnectionClosed){		// On close
      Serial.println("[WS] Closed -> reconnecting...");		// Log
      while(!client.connect(websockets_server)){ Serial.print("."); delay(500); }		// Retry connect
      client.send(String("{\"cmd\":\"hello\",\"id\":\"") + deviceId() + "\",\"role\":\"" + ROLE + "\"}");		// Identify
      Serial.println("\n[WS] Reconnected");		// Log
    }		// End if
  });		// End onEvent
  Serial.print("[WS] Connecting");		// Log
  while(!client.connect(websockets_server)){ Serial.print("."); delay(300); }		// Connect loop
  Serial.println("\n[WS] Connected");		// Log
  client.send(String("{\"cmd\":\"hello\",\"id\":\"") + deviceId() + "\",\"role\":\"" + ROLE + "\"}");		// Identify
}		// End connectWS

// ============================== MOTORS ==============================		// Section header
void initMotors(){		// Initialize I2C and PCA9685
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);		// Start I2C @ 100 kHz
  pinMode(OE_PIN, OUTPUT);		// OE pin as output
  digitalWrite(OE_PIN, LOW);		// Keep outputs enabled by default
  if(!pca.begin()){ Serial.println("[PCA] init error"); while(true) delay(1000); }		// Init PCA or halt
  pca.setOscillatorFrequency(27000000);		// Set internal oscillator
  pca.setPWMFreq(PWM_FREQ);		// Set PWM frequency
  pinHigh(SLEEP_CH);		// Wake DRV8833 (nSLEEP high)
  setMotorSpeed(L_IN1_CH, L_IN2_CH, 0);		// Stop left motor
  setMotorSpeed(R_IN1_CH, R_IN2_CH, 0);		// Stop right motor
  setMotorSpeed(T_IN1_CH, T_IN2_CH, 0);		// Stop turret
  pca.setPin(IR_LED_CH_N, 0, false);		// Ensure IR N off
  pca.setPin(IR_LED_CH_NE, 0, false);		// Ensure IR NE off
  pca.setPin(IR_LED_CH_E, 0, false);		// Ensure IR E off
  pca.setPin(IR_LED_CH_SE, 0, false);		// Ensure IR SE off
  pca.setPin(IR_LED_CH_S, 0, false);		// Ensure IR S off
  pca.setPin(IR_LED_CH_SW, 0, false);		// Ensure IR SW off
  pca.setPin(IR_LED_CH_W, 0, false);		// Ensure IR W off
  pca.setPin(IR_LED_CH_NW, 0, false);		// Ensure IR NW off
}		// End initMotors

// ============================== SETUP ==============================		// Section header
void setup(){		// Arduino setup
#if defined(ARDUINO_USB_CDC_ON_BOOT)		// USB CDC guard
  delay(200);		// Allow USB to enumerate
#endif		// End guard
  initWifi();		// Connect Wi-Fi
  connectWS();		// Connect WebSocket
  initMotors();		// Init PCA/motors
  initCamera();		// Init camera
  pinMode(IR_PIN, INPUT_PULLUP);		// TSOP input with pull-up
  attachInterrupt(digitalPinToInterrupt(IR_PIN), isrIR, FALLING);		// Count falling edges
  
  
  printRingMap();		// Print mapping
  Serial.println("[SETUP] Ready");		// Log ready
  ledcSetup(BUZZ_CH, BEEP_HZ, BUZZ_CH_BITS);
  ledcAttachPin(BUZZ_PIN, BUZZ_CH);
  ledcWriteTone(BUZZ_CH, 0);
}		// End setup

// =============================== LOOP ===============================		// Section header
void loop(){		// Arduino loop
  client.poll();		// Pump WebSocket
  static unsigned long lastPing = 0;		// Last ping time
  if(millis() - lastPing > 15000UL){ client.ping(); lastPing = millis(); }		// Keepalive ping
  sendCamera();		// Send camera frames if enabled
  emitTick();		// Maintain emitter timing
}		// End loop
