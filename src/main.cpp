// ===== Minimal ESP32-S3 Robot Connector with Camera Streaming (SAFE FOR OTA) =====
// This file keeps networking non-blocking so heartbeats and OTA remain responsive.
// It starts the camera OFF. A "flash" command starts streaming; "stream_off" stops.
//
// Frames are sent as BINARY WebSocket messages containing a single JPEG.
// Your Unity host can treat any binary message as a camera frame for the active robot.

#include <Arduino.h>                        // Core Arduino (setup, loop, millis, etc.)
#include <WiFi.h>                           // Wi-Fi client for ESP32
#include <WiFiUdp.h>                        // UDP for discovery announces
#include <ArduinoWebsockets.h>              // Lightweight WebSocket client
#include "secrets.h"                        // Your Wi-Fi credentials (#define WIFI_SSID / WIFI_PASS)
#include "LedController.h"                  // Non-blocking single NeoPixel LED helper (from earlier)
#include "CameraController.h"               // NEW: camera wrapper (keeps camera off until requested)
#include <ArduinoOTA.h>                       // OTA server for ESP32 (PlatformIO 'espota' uses this)

// Use websockets namespace for brevity.
using namespace websockets;                 // Brings WebsocketsClient, WebsocketsMessage, etc. into scope

// -------- Tunables (keep modest to protect OTA/network) --------
static const uint16_t DISCOVERY_PORT = 30560;  // UDP port for announce/reply (same as Unity)
static const uint32_t ANNOUNCE_MS    = 2000;   // How often to announce in Lobby (ms)
static const uint32_t HEARTBEAT_MS   = 2000;   // How often to send WS heartbeat (ms)
static const uint32_t STREAM_FPS     = 5;      // Camera frames per second target (limit bandwidth)
static const uint32_t STREAM_INTERVAL= 1000 / STREAM_FPS; // Milliseconds between frames

// -------- Global objects --------
WiFiUDP udp;                                  // UDP socket for discovery
WebsocketsClient ws;                          // Single WebSocket client
LedController led;                            // LED controller (pin 48)
CameraController cam;                         // Camera controller (starts OFF)

String robotId;                               // MAC-derived ID (12 hex chars)
String wsUrl;                                 // Server URL we’ll connect to after discovery (ws://x.x.x.x:port/path)

unsigned long lastAnnounce  = 0;              // Time of last announce (ms)
unsigned long lastHeartbeat = 0;              // Time of last heartbeat (ms)
unsigned long nextFrameAt   = 0;              // Scheduler for next camera frame (ms)

bool streaming = false;                        // Whether we are currently sending frames

// -------- Helpers: ID, Wi-Fi, Discovery --------

// Build a 12-hex-char ID from the ESP32 efuse MAC (stable & unique).
static String makeRobotId()                    // Returns uppercase hex without colons
{
  uint64_t mac = ESP.getEfuseMac();            // Read 48-bit MAC
  char buf[13];                                 // 12 chars + NUL
  snprintf(buf, sizeof(buf),
           "%02X%02X%02X%02X%02X%02X",         // Format as 6 bytes in hex
           (uint8_t)(mac >> 40), (uint8_t)(mac >> 32), (uint8_t)(mac >> 24),
           (uint8_t)(mac >> 16), (uint8_t)(mac >> 8),  (uint8_t)(mac >> 0));
  return String(buf);                           // Return as Arduino String
}

// Join Wi-Fi (simple blocking connect; fine for boot).
static void connectWifi()
{
  Serial0.print("[WIFI] Connecting to ");       // Log label
  Serial0.println(WIFI_SSID);                   // Show SSID

  WiFi.mode(WIFI_STA);                          // Station mode (client)
  WiFi.begin(WIFI_SSID, WIFI_PASS);             // Start connecting

  while (WiFi.status() != WL_CONNECTED)         // Loop until connected
  {
    Serial0.print(".");                         // Progress dot
    delay(300);                                 // Small delay
  }

  Serial0.println();                            // Newline after dots
  Serial0.print("[WIFI] Connected. IP=");       // Log label
  Serial0.println(WiFi.localIP());              // Show obtained IP
}

// Start UDP discovery (bind port and reset schedule).
static void startDiscovery()
{
  udp.stop();                                   // Close any previous socket
  if (!udp.begin(DISCOVERY_PORT))               // Bind UDP to port
  {
    Serial0.println("[DISCOVERY] udp.begin failed"); // Log error
    return;                                     // Keep going; we’ll retry later
  }
  lastAnnounce = 0;                             // Force immediate first announce
  Serial0.println("[DISCOVERY] listening");     // Log state
}

// Send one announce every ANNOUNCE_MS (non-blocking).
static void maybeAnnounce()
{
  unsigned long now = millis();                 // Snapshot current time
  if (now - lastAnnounce < ANNOUNCE_MS) return; // Not time yet → exit
  lastAnnounce = now;                           // Record send time

  IPAddress bcast(255,255,255,255);             // Limited broadcast (simple/common)
  String payload = String("{\"robotId\":\"") + robotId + "\",\"callsign\":\"\"}"; // Minimal JSON

  udp.beginPacket(bcast, DISCOVERY_PORT);       // Begin UDP packet to broadcast
  udp.write((const uint8_t*)payload.c_str(), payload.length()); // Append bytes
  udp.endPacket();                              // Send packet

  Serial0.println("[DISCOVERY] announce");      // Log action
}

// Read a single UDP reply and extract "ws" URL, if present.
static bool readDiscoveryReply()
{
  int packetSize = udp.parsePacket();           // Check if a packet arrived
  if (packetSize <= 0) return false;            // No packet → nothing to do

  char buf[256];                                 // Temporary buffer
  int len = udp.read(buf, sizeof(buf) - 1);     // Read bytes
  if (len <= 0) return false;                   // Read failed
  buf[len] = '\0';                              // NUL-terminate for safe string ops

  String s(buf);                                 // Wrap C string
  int key = s.indexOf("\"ws\"");                // Find "ws"
  if (key < 0) return false;                    // Not a WS reply
  int colon = s.indexOf(':', key);              // Find colon after key
  if (colon < 0) return false;                  // Malformed
  int q1 = s.indexOf('"', colon + 1);           // Opening quote of URL
  int q2 = s.indexOf('"', q1 + 1);              // Closing quote of URL
  if (q1 < 0 || q2 < 0) return false;           // Malformed

  wsUrl = s.substring(q1 + 1, q2);              // Extract URL
  Serial0.print("[DISCOVERY] ws=");             // Log label
  Serial0.println(wsUrl);                       // Log URL
  return true;                                  // Success
}

// -------- WebSocket handling --------

// Safely stop streaming and camera (idempotent).
static void stopStream()
{
  if (streaming) {                               // If we think we’re streaming
    streaming = false;                           // Drop streaming flag
  }
  if (cam.isStarted()) {                         // If camera running
    cam.stop();                                  // Power it down / free buffers
    Serial0.println("[CAM] stopped");            // Log state
  }
}

// Start streaming (idempotent). Returns true if ready to stream frames.
// --- replace the whole startStream() with this ---
static bool startStream()
{
  if (!cam.isStarted())                          // If camera not running yet
  {
    cam.begin();                                 // Configure pins/params (no return value)

    if (!cam.start())                            // Power up + init; returns bool
    {
      Serial0.println("[CAM] start failed");     // Log failure once
      return false;                              // Do not enable streaming
    }
    Serial0.println("[CAM] started");            // Log success
  }
  streaming   = true;                            // We’re now allowed to send frames
  nextFrameAt = millis();                        // Schedule first frame immediately
  return true;                                   // All good
}


// Process an incoming text message (commands).
static void handleWsText(const String& s)
{
  // Log small messages for visibility (optional).
  Serial0.print("[WS] RX: ");                    // Log prefix
  Serial0.println(s);                            // Log payload

  // If Unity sends {"cmd":"flash","pin":48,"ms":2000} we start LED + camera streaming.
  if (s.indexOf("\"cmd\":\"flash\"") >= 0)       // Check for flash command
  {
    // Extract optional pin / ms (keep liberal parsing).
    int pinVal = 48;                             // Default LED pin
    int msVal  = 2000;                           // Default flash duration
    int pIdx = s.indexOf("\"pin\"");             // Find "pin"
    if (pIdx >= 0) { int c = s.indexOf(':', pIdx); if (c >= 0) pinVal = s.substring(c+1).toInt(); } // Parse pin
    int mIdx = s.indexOf("\"ms\"");              // Find "ms"
    if (mIdx >= 0) { int c = s.indexOf(':', mIdx); if (c >= 0) msVal  = s.substring(c+1).toInt(); } // Parse ms

    // Start LED flash non-blocking when it targets our pin.
    if (pinVal == 48)                            // Only act on our LED pin
    {
      led.startFlash((uint32_t)msVal);           // Blink green/white for msVal without blocking
    }

    // ALSO: begin camera stream for this robot.
    startStream();                               // Start (or no-op) camera + enable streaming
    return;                                      // Done handling
  }

  // If Unity sends {"cmd":"stream_off"} we stop streaming and power down the camera.
  if (s.indexOf("\"cmd\":\"stream_off\"") >= 0)  // Check for stop stream command
  {
    stopStream();                                // Disable streaming and stop sensor
    return;                                      // Done
  }

  // You can extend with {"cmd":"stream_on"} explicit start if you prefer.
  if (s.indexOf("\"cmd\":\"stream_on\"") >= 0)   // Optional explicit start command
  {
    startStream();                               // Start streaming
    return;                                      // Done
  }

  // Unknown command → ignore (keeps robustness).
}

// Connect (or reconnect) WebSocket using wsUrl (simple retry).
static void connectWebSocket()
{
  if (wsUrl.length() == 0)                       // Without a URL we cannot connect
  {
    Serial0.println("[WS] no url");              // Log and bail
    return;                                      // Exit
  }

  ws.close();                                    // Ensure previous socket closed
  delay(150);                                    // Small settle delay

  // Message handler: text OR binary frames.
  ws.onMessage([](WebsocketsMessage msg)         // Register message callback
  {
    if (msg.isText())                            // Text frames carry commands
    {
      handleWsText(msg.data());                  // Parse a few simple JSON-like commands
    }
    else                                         // Binary frames are ignored on robot
    {
      // (We only SEND binary to the server; receiving binary isn’t used.)
    }
  });

  // Event handler: open/close/ping/pong.
  ws.onEvent([](WebsocketsEvent e, String)       // Register event callback
  {
    if (e == WebsocketsEvent::ConnectionOpened)  // On open
    {
      Serial0.println("[WS] opened");            // Log
      String hello = String("{\"cmd\":\"hello\",\"id\":\"") + robotId + "\"}"; // Hello handshake
      ws.send(hello);                             // Send hello
      lastHeartbeat = 0;                          // Reset heartbeat schedule
    }
    else if (e == WebsocketsEvent::ConnectionClosed) // On close
    {
      Serial0.println("[WS] closed");            // Log
      stopStream();                               // Make sure camera is off when disconnected
      wsUrl = "";                                 // Force return to discovery
      startDiscovery();                           // Reopen UDP
    }
    else if (e == WebsocketsEvent::GotPing)      // Ping from server
    {
      // (Library auto-pongs; nothing to do.)
    }
    else if (e == WebsocketsEvent::GotPong)      // Pong from server
    {
      // (Optional debug.)
    }
  });

  Serial0.print("[WS] connecting ");             // Log prefix
  Serial0.println(wsUrl);                        // Log URL
  while (!ws.connect(wsUrl.c_str()))             // Retry until connected
  {
    Serial0.print(".");                          // Dot per attempt
    delay(300);                                  // Short delay between tries
  }
  Serial0.println();                             // Newline after dots
  Serial0.println("[WS] connected");             // Log success
}

// -------- Frame sending (non-blocking) --------

// Send one JPEG frame as a single binary WS message (returns immediately).
static void sendOneFrameIfDue(uint32_t nowMs)
{
  if (!streaming) return;                        // Not streaming → nothing to do
  if ((int32_t)(nextFrameAt - nowMs) > 0) return;// Not time yet per fps limiter

  nextFrameAt = nowMs + STREAM_INTERVAL;         // Schedule next frame slot

  if (!ws.available())                           // If WS not up, skip capture
    return;                                      // Avoid wasted work

  camera_fb_t* fb = cam.capture();               // Capture a JPEG frame buffer
  if (!fb)                                       // If capture failed
  {
    // Skip this slot silently; could log if repeated.
    return;                                      // Return to loop
  }

  // Send the JPEG as one binary message (Unity: handle as a camera frame).
  ws.sendBinary((const char*)fb->buf, fb->len);  // Transmit JPEG payload (non-blocking in lib)

  esp_camera_fb_return(fb);                      // Return buffer to driver (ALWAYS do this)
}

// --- OTA setup: starts the OTA server and ensures camera/WS won't interfere ---
static void setupOTA()                        // Call once after Wi-Fi is connected
{
  ArduinoOTA.setPort(3232);                   // Use the same port your PIO log shows (3232)

  // Build a readable hostname that includes your robotId (helps when multiple bots are online).
  String host = String("thunder-") + robotId; // e.g., "thunder-14AEC64EB580"
  ArduinoOTA.setHostname(host.c_str());       // Set OTA hostname (optional but nice)

  ArduinoOTA.setPassword("thunder123");       // Must match your PIO 'auth' in the log

  // When OTA starts, stop anything that could contend for CPU/PSRAM/WiFi (camera/WS/etc).
  ArduinoOTA.onStart([]()                     // Called once when upload begins
  {
    Serial0.println("[OTA] Start");           // Log OTA start
    stopStream();                             // Ensure camera is powered down & streaming disabled
    // Optional: if you want to fully isolate OTA, you can also close WS:
    // if (ws.available()) ws.close();        // Not strictly required, OTA works fine with WS idle
  });

  // Optional: progress log (handy when debugging timeouts).
  ArduinoOTA.onProgress([](unsigned int prog, unsigned int total)
  {
    // Keep this lightweight; just a terse progress every ~5%.
    static unsigned long lastPct = 101;       // Force first print
    unsigned long pct = (prog * 100UL) / total; // Compute percent
    if (pct != lastPct && (pct % 5 == 0))     // Throttle prints
    {
      lastPct = pct;                          // Remember last percent shown
      Serial0.print("[OTA] ");                // Prefix
      Serial0.print(pct);                     // Percent
      Serial0.println("%");                   // Suffix
    }
  });

  // End/error hooks just for visibility.
  ArduinoOTA.onEnd([]()
  {
    Serial0.println("[OTA] End");             // Upload finished (device will auto-apply & reboot)
  });

  ArduinoOTA.onError([](ota_error_t err)
  {
    Serial0.print("[OTA] Error=");            // Show error code
    Serial0.println((int)err);                // Cast to int for serial
  });

  ArduinoOTA.begin();                         // Start listening on TCP:3232
  Serial0.print("[OTA] Ready on ");           // Log readiness with IP + port
  Serial0.print(WiFi.localIP());              // Device IP
  Serial0.println(":3232");                   // Port
}

// -------- Arduino setup / loop --------


#include "esp_heap_caps.h"

static void dump_mem(const char* tag) {
  size_t iram  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  size_t dram  = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t spir  = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  Serial0.printf("[MEM] %s  DRAM=%u  IRAM=%u  SPIRAM=%u  total=%u\n",
                 tag, (unsigned)dram, (unsigned)iram, (unsigned)spir,
                 (unsigned)esp_get_free_heap_size());
}

void setup()
{
  Serial0.begin(115200);                         // USB CDC serial for logs
  delay(100);                                    // Tiny settle delay


  Serial0.println("[BOOT] ESP32-S3 robot w/ camera"); // Banner

  led.begin(48);                                 // Init NeoPixel on GPIO48 (non-blocking)
  cam.begin();                                   // Prepare camera config ONLY (no power yet)

  robotId = makeRobotId();                       // Build stable ID
  Serial0.print("[ID] ");                        // Log label
  Serial0.println(robotId);                      // Log ID

  connectWifi();                                 // Join Wi-Fi
  setupOTA();                                  // Start OTA server (must be after Wi-Fi is up)
  startDiscovery();                               // Begin UDP listening + announce cadence

  // NOTE: We do NOT start the camera here, to keep boot light and OTA unaffected.
}

void loop()
{
  uint32_t now = millis();                       // Snapshot current time once per tick

  ArduinoOTA.handle();                         // <-- Let OTA process its TCP/UDP work (non-blocking)

  // If we don't yet have a WS URL, we are in discovery mode.
  if (wsUrl.length() == 0)                       // No URL → still discovering
  {
    maybeAnnounce();                             // Send UDP announce if due
    if (readDiscoveryReply())                    // If we got a WS URL
    {
      connectWebSocket();                        // Connect to that URL
    }
    // Keep LED animation alive even in discovery.
    led.update(now);                             // Advance LED effect
    delay(10);                                   // Tiny sleep to yield CPU/Wi-Fi
    return;                                      // Done with this tick
  }

  // Once we have a URL, we stay connected or fall back to discovery.
  if (ws.available())                            // If WS is connected
  {
    ws.poll();                                   // Service inbound events/messages
    if (now - lastHeartbeat >= HEARTBEAT_MS)     // If time to heartbeat
    {
      lastHeartbeat = now;                       // Record send time
      String hb = String("{\"cmd\":\"hb\",\"t\":") + String((unsigned long)now) + "}"; // Tiny JSON
      ws.send(hb);                                // Send heartbeat
    }
    sendOneFrameIfDue(now);                      // Maybe send one camera frame
  }
  else                                           // WS disconnected
  {
    stopStream();                                 // Ensure camera is off when link drops
    wsUrl = "";                                   // Clear URL to re-enter discovery
    startDiscovery();                              // Restart UDP
  }

  led.update(now);                                // Keep LED animation responsive
  delay(5);                                       // Small delay to avoid tight spin
}
