// main.cpp — ESP32-S3 robot client
// Features:
// - Wi-Fi + UDP discovery to find Unity server
// - WebSocket client to exchange commands and video
// - OTA updates (safe: camera + motors disabled during OTA)
// - Camera streaming (VGA JPEG) with a simple pacing loop
// - LED flash on GPIO 48 (non-blocking; handled by LedController)
// - Motor control via PCA9685 over I²C (Adafruit driver)
//   * SDA = 21, SCL = 47 (do NOT reuse camera pins)
//   * OE  = GPIO 41 (LOW enables PCA outputs)
//   * DRV8833 nSLEEP = PCA9685 channel 14 (HIGH wakes drivers)
// Keep everything very simple and comment every line.

#include <Arduino.h>                      // Include Arduino core
#include <WiFi.h>                         // Include Wi-Fi support
#include <WiFiUdp.h>                      // Include UDP sockets
#include <ArduinoWebsockets.h>            // Include WebSocket client
#include <ArduinoOTA.h>                   // Include OTA update support
#include "secrets.h"                      // Include Wi-Fi credentials (WIFI_SSID / WIFI_PASS)
#include "LedController.h"                // Include simple non-blocking LED helper (GPIO 48)
#include "CameraController.h"             // Include camera wrapper (VGA JPEG; non-blocking API)
#include "MotorController_PCA9685.h"      // Include PCA9685 motor controller (with nSLEEP on ch14)

using namespace websockets;               // Use shorter names for websockets types

// ---------- Tunable timing (simple constants for clarity) ----------
static const uint16_t DISCOVERY_PORT = 30560;   // UDP port used for discovery
static const uint32_t ANNOUNCE_MS    = 2000;    // How often we announce over UDP
static const uint32_t HEARTBEAT_MS   = 2000;    // How often we heartbeat over WebSocket
static const uint32_t STREAM_INTERVAL_MS = 40;  // Target camera frame interval (≈25 FPS)

// ---------- PCA9685 wiring (your hardware) ----------
static const int PCA_SDA_PIN = 21;              // I²C SDA pin to PCA9685
static const int PCA_SCL_PIN = 47;              // I²C SCL pin to PCA9685
static const int PCA_OE_PIN  = 41;              // PCA9685 OE pin (GPIO 41) — drive LOW to enable outputs
static const uint8_t  PCA_ADDR   = 0x40;        // PCA9685 I²C address
static const float    PCA_PWM_HZ = 1000.0f;     // PWM frequency for DC motors (1 kHz)

// ---------- Globals: simple singletons for this sketch ----------
WiFiUDP udp;                                    // UDP socket for discovery
WebsocketsClient ws;                            // WebSocket client
LedController led;                              // LED helper (GPIO 48)
CameraController cam;                           // Camera helper (inlines esp_camera usage)
MotorController_PCA9685 motors;                 // PCA9685 motor driver (handles nSLEEP on ch14)

// ---------- Simple state flags and clocks ----------
String robotId;                                 // Our unique ID (derived from MAC)
String wsUrl;                                   // Server WebSocket URL learned from discovery
unsigned long lastAnnounce  = 0;                // Last time we sent a UDP announce
unsigned long lastHeartbeat = 0;                // Last time we sent a WS heartbeat
unsigned long nextStreamAt  = 0;                // Next time we are allowed to send a frame
bool g_wsOpen         = false;                  // Whether the WS is open
bool desiredStreaming = false;                  // Whether server wants us to stream
uint32_t g_framesSent = 0;                      // Counter for debug prints
bool g_motorsEnabled  = false;                  // Whether motors are enabled by server

// ---------- Helper: build a 12-char uppercase MAC ID ----------
static String makeRobotId() {
  // Read the 48-bit MAC from efuse
  uint64_t mac = ESP.getEfuseMac();                                 // Read unique MAC
  // Prepare a small buffer to format hex bytes
  char buf[13];                                                      // 12 hex chars + NUL
  // Format to uppercase hex without separators
  snprintf(buf, sizeof(buf), "%02X%02X%02X%02X%02X%02X",            // Format string
           (uint8_t)(mac >> 40), (uint8_t)(mac >> 32),              // Byte 5, 4
           (uint8_t)(mac >> 24), (uint8_t)(mac >> 16),              // Byte 3, 2
           (uint8_t)(mac >>  8), (uint8_t)(mac >>  0));             // Byte 1, 0
  // Return a String for easy JSON concat
  return String(buf);                                                // Return ID
}

// ---------- Connect to Wi-Fi (very basic blocking loop for clarity) ----------
static void connectWifi() {
  // Print the SSID we are trying to join
  Serial0.print("[WIFI] Connecting ");                               // Log prefix
  Serial0.println(WIFI_SSID);                                        // Show SSID
  // Put radio in STA mode
  WiFi.mode(WIFI_STA);                                               // Station mode
  // Start connecting
  WiFi.begin(WIFI_SSID, WIFI_PASS);                                  // Begin association
  // Wait until connected (simple loop with dots)
  while (WiFi.status() != WL_CONNECTED) {                            // Loop while not connected
    Serial0.print(".");                                              // Progress dot
    delay(300);                                                      // Small delay to avoid tight loop
  }
  // Newline after dots
  Serial0.println();                                                 // Newline
  // Show acquired IP
  Serial0.print("[WIFI] IP=");                                       // Label
  Serial0.println(WiFi.localIP());                                   // IP address
}

// ---------- Setup OTA (disable camera + motors while updating) ----------
static void setupOTA() {
  // Listen on the standard ArduinoOTA port
  ArduinoOTA.setPort(3232);                                          // OTA port
  // Build a friendly hostname
  String host = String("thunder-") + robotId;                        // Hostname
  ArduinoOTA.setHostname(host.c_str());                              // Apply hostname
  // Use a simple password
  ArduinoOTA.setPassword("thunder123");                              // Password

  // When OTA starts, stop motion and video for safety
  ArduinoOTA.onStart([](){                                           // On OTA start callback
    Serial0.println("[OTA] Start");                                  // Log
    desiredStreaming = false;                                        // Clear streaming flag
    cam.stop();                                                      // Stop camera if running
    motors.enable(false);                                            // Disable motors (also nSLEEP low)
    g_motorsEnabled = false;                                         // Mirror state flag
  });

  // When OTA ends, just log for confirmation
  ArduinoOTA.onEnd([](){                                             // On OTA end callback
    Serial0.println("[OTA] End");                                    // Log
  });

  // On OTA error, print error code
  ArduinoOTA.onError([](ota_error_t e){                               // On error callback
    Serial0.print("[OTA] Error=");                                    // Label
    Serial0.println((int)e);                                          // Error code
  });

  // Start the OTA service
  ArduinoOTA.begin();                                                // Begin OTA
  // Print where OTA will be reachable
  Serial0.print("[OTA] Ready ");                                     // Label
  Serial0.print(WiFi.localIP());                                     // IP
  Serial0.println(":3232");                                          // Port suffix
}

// ---------- Start UDP discovery listener (for replies) ----------
static void startDiscovery() {
  // Ensure any previous UDP is closed
  udp.stop();                                                        // Close if open
  // Bind to the discovery port
  if (!udp.begin(DISCOVERY_PORT)) {                                  // Try to bind
    Serial0.println("[DISCOVERY] udp.begin failed");                 // Log failure
    return;                                                          // Bail if cannot bind
  }
  // Force an immediate announce on next loop
  lastAnnounce = 0;                                                  // Reset announce timer
  // Log that we are listening
  Serial0.println("[DISCOVERY] listening");                          // Debug
}

// ---------- Periodically announce (very small JSON) ----------
static void maybeAnnounce() {
  // Grab current time
  const unsigned long now = millis();                                // Current ms
  // If not yet time to announce, skip
  if (now - lastAnnounce < ANNOUNCE_MS) return;                      // Respect cadence
  // Stamp the time
  lastAnnounce = now;                                                // Update last announce
  // Use limited broadcast address
  IPAddress bcast(255,255,255,255);                                  // Broadcast
  // Prepare tiny JSON with our robotId and empty callsign
  String payload = String("{\"robotId\":\"") + robotId + "\",\"callsign\":\"\"}"; // JSON text
  // Begin UDP packet to the broadcast address
  udp.beginPacket(bcast, DISCOVERY_PORT);                            // Open packet
  // Write JSON bytes
  udp.write((const uint8_t*)payload.c_str(), payload.length());      // Send payload
  // Finalize packet
  udp.endPacket();                                                   // Close packet
  // Log
  Serial0.println("[DISCOVERY] announce");                           // Debug
}

// ---------- Read one UDP reply and extract the ws URL (very simple parse) ----------
static bool readDiscoveryReply() {
  // Parse incoming UDP packet
  int sz = udp.parsePacket();                                        // Size or 0
  // If nothing, return false
  if (sz <= 0) return false;                                         // No data
  // Buffer to hold message
  char buf[256];                                                     // Small buffer
  // Read bytes into buffer
  int n = udp.read(buf, sizeof(buf)-1);                              // Read data
  // If read error, return false
  if (n <= 0) return false;                                          // Error
  // Null-terminate buffer
  buf[n] = '\0';                                                     // NUL terminate
  // Wrap in String for easy search
  String s(buf);                                                     // Convert to String
  // Find the "ws":"..." field
  int k = s.indexOf("\"ws\"");                                       // Find key
  if (k < 0) return false;                                           // Not present
  // Find colon after key
  int c = s.indexOf(':', k);                                         // Find colon
  if (c < 0) return false;                                           // Malformed
  // Find the opening quote of the value
  int q1 = s.indexOf('"', c+1);                                      // First quote
  // Find the closing quote of the value
  int q2 = s.indexOf('"', q1+1);                                     // Second quote
  // Validate quotes
  if (q1 < 0 || q2 < 0) return false;                                // Malformed
  // Extract the URL substring
  wsUrl = s.substring(q1+1, q2);                                     // Save URL
  // Log the URL
  Serial0.print("[DISCOVERY] ws=");                                  // Label
  Serial0.println(wsUrl);                                            // URL
  // Indicate we got something
  return true;                                                       // Success
}

// ---------- Ensure camera fully stopped (idempotent) ----------
static void ensureStreamStop() {
  // If camera is started, stop it
  if (cam.isStarted()) cam.stop();                                   // Stop camera safely
}

// ---------- Ensure camera started (idempotent) ----------
static void ensureStreamStart() {
  // If camera not started yet, start it
  if (!cam.isStarted()) {                                            // Not started
    // Try to start the camera
    if (!cam.start()) {                                              // Start camera
      // If start fails, log and clear streaming desire
      Serial0.println("[CAM] start failed (send path)");             // Error
      desiredStreaming = false;                                      // Back off
    }
  }
}

// ---------- Try to send a single JPEG frame (keeps it very simple) ----------
static void maybeSendFrame(const unsigned long now) {
  // Must be connected and asked to stream
  if (!g_wsOpen) return;                                             // Need WS
  if (!desiredStreaming) return;                                     // Need request
  // Ensure camera is running
  ensureStreamStart();                                               // Start if needed
  // Bail if camera is still not available
  if (!cam.isStarted()) return;                                      // If still off, exit
  // Delay a bit around heartbeat to avoid starving it
  const long msToHb = (long)((lastHeartbeat + HEARTBEAT_MS) - now);  // Time to next HB
  if (msToHb <= 120) return;                                         // Skip if HB is imminent
  // Respect frame pacing
  if ((long)(nextStreamAt - now) > 0) return;                        // Not time yet
  // Schedule next frame time
  nextStreamAt = now + STREAM_INTERVAL_MS;                           // Next allowed frame
  // Ask camera for a frame buffer
  camera_fb_t* fb = esp_camera_fb_get();                             // Get frame
  // If none, log and exit
  if (!fb) { Serial0.println("[CAM] fb_get NULL"); return; }         // No frame
  // Get size of JPEG
  const size_t n = fb->len;                                          // Byte length
  // Send as a single binary WebSocket message
  bool ok = ws.sendBinary((const char*)fb->buf, n);                  // Send
  // Count frames for periodic logging
  g_framesSent++;                                                    // Increment
  // Print error if send failed
  if (!ok) Serial0.printf("[CAM] sendBinary failed (len=%u)\n", (unsigned)n); // Error
  // Print every 10 frames as a light diagnostic
  else if ((g_framesSent % 10) == 0) Serial0.printf("[CAM] sent frames=%u last=%u bytes\n", (unsigned)g_framesSent, (unsigned)n); // Info
  // Return buffer to camera driver
  esp_camera_fb_return(fb);                                          // Release frame
}

// ---------- Handle a single text WS message (very simple "parsing") ----------
static void handleWsText(const String& s) {
  // Print the raw JSON for debugging
  Serial0.print("[WS] RX: ");                                        // Label
  Serial0.println(s);                                                // JSON

  // If this is a "flash" command, parse pin+ms and trigger LED flash, also enable streaming
  if (s.indexOf("\"cmd\":\"flash\"") >= 0) {                         // Look for flash
    // Default values
    int pinVal = 48;                                                 // Default pin
    int msVal  = 2000;                                               // Default duration
    // Try to find "pin" value (very simple parse)
    int p = s.indexOf("\"pin\""); if (p >= 0) { int c = s.indexOf(':', p); if (c >= 0) pinVal = s.substring(c+1).toInt(); } // Parse pin
    // Try to find "ms" value (very simple parse)
    int m = s.indexOf("\"ms\"");  if (m >= 0) { int c = s.indexOf(':', m); if (c >= 0)  msVal  = s.substring(c+1).toInt(); } // Parse duration
    // If pin is our LED (GPIO 48), start a non-blocking flash
    if (pinVal == 48) led.startFlash((uint32_t)msVal);               // Start flash
    // Also request camera stream start
    desiredStreaming = true;                                         // Ask to stream
    // Log stream desire
    Serial0.println("[CAM] desiredStreaming = true (flash)");        // Info
    // Done with this command
    return;                                                          // Exit
  }

  // Start streaming command
  if (s.indexOf("\"cmd\":\"stream_on\"")  >= 0) {                    // Stream on
    desiredStreaming = true;                                         // Set flag
    Serial0.println("[CAM] desiredStreaming = true (cmd)");          // Log
    return;                                                          // Exit
  }

  // Stop streaming command
  if (s.indexOf("\"cmd\":\"stream_off\"") >= 0) {                    // Stream off
    desiredStreaming = false;                                        // Clear flag
    Serial0.println("[CAM] desiredStreaming = false (cmd)");         // Log
    return;                                                          // Exit
  }

  // Motors ON command (enable software gate and wake DRV8833 via nSLEEP)
  if (s.indexOf("\"cmd\":\"motors_on\"")  >= 0) {                    // Motors on
    motors.enable(true);                                             // Enable driver (also nSLEEP HIGH)
    g_motorsEnabled = true;                                          // Mirror flag
    Serial0.printf("[MOT] motors ON (isEnabled=%d)\n", motors.isEnabled()); // Confirm
    return;                                                          // Exit
  }

  // Motors OFF command (disable software gate and sleep DRV8833 via nSLEEP)
  if (s.indexOf("\"cmd\":\"motors_off\"") >= 0) {                    // Motors off
    motors.enable(false);                                            // Disable driver (also nSLEEP LOW)
    g_motorsEnabled = false;                                         // Mirror flag
    Serial0.printf("[MOT] motors OFF (isEnabled=%d)\n", motors.isEnabled()); // Confirm
    return;                                                          // Exit
  }

  // Drive command: {"cmd":"drive","l":-1..1,"r":-1..1}
  if (s.indexOf("\"cmd\":\"drive\"") >= 0) {                         // Drive
    // Defaults
    float l = 0, r = 0;                                              // Defaults
    // Extract "l" value (very simple parse)
    int il = s.indexOf("\"l\""); if (il >= 0) { int c = s.indexOf(':', il); if (c >= 0) l = s.substring(c+1).toFloat(); } // Parse l
    // Extract "r" value (very simple parse)
    int ir = s.indexOf("\"r\""); if (ir >= 0) { int c = s.indexOf(':', ir); if (c >= 0) r = s.substring(c+1).toFloat(); } // Parse r
    // Clamp to [-1..1] to be safe
    l = fmaxf(-1.f, fminf(1.f, l));                                  // Clamp l
    r = fmaxf(-1.f, fminf(1.f, r));                                  // Clamp r
    // Log what we received and whether motors are enabled
    Serial0.printf("[MOT] drive l=%.3f r=%.3f (enabled=%d)\n", l, r, motors.isEnabled()); // Debug
    // Apply to motors (does nothing if not enabled)
    motors.setLeftRight(l, r);                                       // Drive L/R
    // Exit
    return;                                                          // Done
  }

// Turret command: accept {"cmd":"turret","speed":-1..1} OR {"cmd":"turret","v":-1..1}
if (s.indexOf("\"cmd\":\"turret\"") >= 0) {        // Detect a turret command
  float v = 0.0f;                                  // Default to 0 if no value found

  // Try to find "speed" first (matches your Unity sender)
  int is = s.indexOf("\"speed\"");                 // Look for "speed"
  if (is >= 0) {                                   // If found
    int c = s.indexOf(':', is);                    // Find the colon after "speed"
    if (c >= 0) v = s.substring(c + 1).toFloat();  // Convert the substring to float
  } else {
    // Fallback to legacy "v" key if "speed" is not present
    int iv = s.indexOf("\"v\"");                   // Look for "v"
    if (iv >= 0) {                                 // If found
      int c = s.indexOf(':', iv);                  // Find the colon after "v"
      if (c >= 0) v = s.substring(c + 1).toFloat();// Convert the substring to float
    }
  }

  // Clamp to the valid range [-1..+1]
  v = fmaxf(-1.0f, fminf(1.0f, v));               // Keep value in range

  // Log the parsed value and whether motors are enabled
  Serial0.printf("[MOT] turret v=%.3f (enabled=%d)\n", v, motors.isEnabled());

  // Apply to turret H-bridge (does nothing if not enabled)
  motors.setTurret(v);                              // Drive turret channels

  // Done handling this message
  return;
}
}

// ---------- Connect WebSocket (simple blocking retry) ----------
static void connectWebSocket() {
  // If we have no URL yet, bail
  if (wsUrl.isEmpty()) {                                             // No URL
    Serial0.println("[WS] no url");                                  // Log
    return;                                                          // Exit
  }
  // Ensure previous WS is closed
  ws.close();                                                        // Close any prior socket
  // Small delay to allow clean close
  delay(150);                                                        // Short delay
  // Handle incoming messages
  ws.onMessage([](WebsocketsMessage msg){                            // Message callback
    if (msg.isText()) handleWsText(msg.data());                      // Only parse text JSON
  });
  // Handle WS events (open/close)
  ws.onEvent([](WebsocketsEvent e, String){                           // Event callback
    if (e == WebsocketsEvent::ConnectionOpened) {                    // On open
      Serial0.println("[WS] Opened");                                // Log
      g_wsOpen = true;                                               // Flag open
      String hello = String("{\"cmd\":\"hello\",\"id\":\"") + robotId + "\"}"; // Hello JSON
      ws.send(hello);                                                // Send hello
      lastHeartbeat = 0;                                             // Reset heartbeat timer
    } else if (e == WebsocketsEvent::ConnectionClosed) {             // On close
      Serial0.println("[WS] Closed -> rediscover");                  // Log
      g_wsOpen = false;                                              // Flag closed
      desiredStreaming = false;                                      // Clear streaming wish
      ensureStreamStop();                                            // Stop camera
      motors.enable(false);                                          // Disable motors (nSLEEP low)
      g_motorsEnabled = false;                                       // Mirror flag
      wsUrl.clear();                                                 // Clear URL to force discovery
      startDiscovery();                                              // Rebind UDP for next server
    }
  });
  // Log where we are connecting
  Serial0.print("[WS] connecting ");                                  // Label
  Serial0.println(wsUrl);                                             // URL
  // Keep trying until connected (simple approach)
  while (!ws.connect(wsUrl.c_str())) {                                // Try connect
    Serial0.print(".");                                               // Progress dot
    delay(300);                                                       // Backoff a bit
  }
  // Newline after dots
  Serial0.println();                                                  // Newline
  // Confirm connected
  Serial0.println("[WS] connected");                                  // Log
}

// ---------- Arduino setup (one-time init) ----------
void setup() {
  // Start serial console
  Serial0.begin(115200);                                              // Serial @115200
  // Small settle delay
  delay(100);                                                         // Delay
  // Banner
  Serial0.println("[BOOT] ESP32-S3 robot");                           // Banner

  // Prepare LED controller on GPIO 48
  led.begin(48);                                                      // Init LED helper

  // Prepare camera (pins and config inside CameraController.h; using VGA JPEG)
  cam.begin();                                                        // Init camera struct (no start yet)

  // Prepare PCA9685 motor driver (I²C + OE + nSLEEP handling inside)
  motors.begin(PCA_ADDR, PCA_PWM_HZ, PCA_SDA_PIN, PCA_SCL_PIN, PCA_OE_PIN); // Init motor driver
  motors.enable(false);                                               // Keep motors disabled at boot

  // Build our robot ID from MAC
  robotId = makeRobotId();                                            // Compute ID
  // Print the ID
  Serial0.print("[ID] ");                                             // Label
  Serial0.println(robotId);                                           // Print ID

  // Connect to Wi-Fi
  connectWifi();                                                      // Join network

  // Start OTA server (safe guards inside)
  setupOTA();                                                         // Enable OTA

  // Begin discovery listener
  startDiscovery();                                                   // Bind UDP port
}

// ---------- Arduino loop (very small state machine) ----------
void loop() {
  // Snapshot current time
  const unsigned long now = millis();                                 // Milliseconds since boot

  // Service OTA regularly
  ArduinoOTA.handle();                                                // Allow OTA updates

  // If we do not have a server URL yet, keep announcing and listening
  if (wsUrl.isEmpty()) {                                              // Not discovered yet
    maybeAnnounce();                                                  // Periodic announce
    if (readDiscoveryReply()) connectWebSocket();                     // If reply parsed, connect WS
    led.update(now);                                                  // Update LED animation
    delay(5);                                                         // Light yield
    return;                                                           // Next tick
  }

  // If we have a WS URL, pump WS events
  ws.poll();                                                          // Service WebSocket

  // If socket is open, run heartbeat + video
  if (g_wsOpen) {                                                     // Connected
    // Send heartbeat at fixed cadence
    if (now - lastHeartbeat >= HEARTBEAT_MS) {                        // Time for HB?
      lastHeartbeat = now;                                            // Stamp
      String hb = String("{\"cmd\":\"hb\",\"t\":") + String((unsigned long)now) + "}"; // Tiny JSON
      ws.send(hb);                                                    // Send heartbeat
    }
    // Attempt to send a camera frame (pacing + HB spacing)
    maybeSendFrame(now);                                              // Video path
  }
  // If socket is not open, clean up and restart discovery
  else {                                                              // Not connected
    ensureStreamStop();                                               // Make sure camera is off
    motors.enable(false);                                             // Ensure motors disabled
    g_motorsEnabled = false;                                          // Mirror flag
    desiredStreaming = false;                                         // Clear streaming wish
    wsUrl.clear();                                                    // Clear URL to trigger discovery
    startDiscovery();                                                 // Restart discovery
  }

  // Keep LED animation responsive
  led.update(now);                                                    // Update LED
  // Small delay to be gentle on CPU
  delay(5);                                                           // Yield
}
