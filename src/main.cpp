// ===== Minimal ESP32(S3) Robot Connector (BEGINNER-FRIENDLY, HEAVILY COMMENTED) =====
// What this does:
// 1) Join Wi-Fi using credentials in secrets.h
// 2) Broadcast a small UDP "announce" packet every 2 seconds on port 30560
// 3) Wait for a UDP reply that contains a WebSocket URL (e.g. {"ws":"ws://IP:8080/esp32"})
// 4) Connect a WebSocket client to that URL
// 5) Send a "hello" message once connected, then send a heartbeat every 2 seconds
// 6) If WebSocket disconnects, clear URL and go back to step 2

#include <Arduino.h>                // Basic Arduino functions (setup, loop, etc.)
#include <WiFi.h>                   // Wi-Fi support for ESP32
#include <WiFiUdp.h>                // UDP networking support
#include <ArduinoWebsockets.h>      // Simple WebSocket client library
#include "secrets.h"                // You create this file with: #define WIFI_SSID "..." and #define WIFI_PASS "..."

using namespace websockets;         // Makes WebSockets classes easier to reference

// ---- Configuration you can tweak ----
static const uint16_t DISCOVERY_PORT = 30560;   // UDP port used for announce + reply
static const uint32_t ANNOUNCE_MS    = 2000;    // How often we broadcast an announce (milliseconds)
static const uint32_t HEARTBEAT_MS   = 2000;    // How often we send a heartbeat over WS (milliseconds)

// ---- Global objects and state ----
WiFiUDP udp;                         // UDP socket object
WebsocketsClient ws;                 // Single global WebSocket client (simple + stable)
String robotId;                      // Our unique device ID (we’ll use the MAC as hex)
String wsUrl;                        // WebSocket URL we’ll connect to after discovery (e.g., "ws://192.168.0.10:8080/esp32")
unsigned long lastAnnounce = 0;      // Time of last UDP announce (millis)
unsigned long lastHeartbeat = 0;     // Time of last WS heartbeat (millis)

// ----- Helper: get unique ID from ESP32’s MAC in hex (e.g. "14AEC64EB580") -----
String getRobotId()
{
  // Read 48-bit MAC from efuse (built-in unique ID)
  uint64_t mac = ESP.getEfuseMac();                           // 64-bit value; we use lower 48 bits
  char buf[13];                                               // 12 hex chars + null terminator
  snprintf(                                                   // Format as 12 hex characters
    buf, sizeof(buf),
    "%02X%02X%02X%02X%02X%02X",
    (uint8_t)(mac >> 40), (uint8_t)(mac >> 32), (uint8_t)(mac >> 24),
    (uint8_t)(mac >> 16), (uint8_t)(mac >> 8),  (uint8_t)(mac >> 0)
  );
  return String(buf);                                         // Return as Arduino String
}

// ----- Wi-Fi connect (blocking, simple) -----
void connectWifi()
{
  Serial0.print("[WIFI] Connecting to ");
  Serial0.println(WIFI_SSID);                                 // Show which SSID we try

  WiFi.mode(WIFI_STA);                                        // Station mode (we are a client)
  WiFi.begin(WIFI_SSID, WIFI_PASS);                           // Start connecting

  while (WiFi.status() != WL_CONNECTED)                       // Wait until connected
  {
    Serial0.print(".");                                       // Dots = waiting
    delay(300);                                               // Small pause between checks
  }

  Serial0.print("\n[WIFI] IP: ");                             // Print the IP we got from router
  Serial0.println(WiFi.localIP());                            // Show IP address
}

// ----- Start UDP discovery (open socket on DISCOVERY_PORT) -----
void startDiscovery()
{
  Serial0.println("[DISCOVERY] Starting UDP announce loop");  // Info message
  udp.begin(DISCOVERY_PORT);                                  // Bind local UDP port so we can send/receive
  lastAnnounce = 0;                                           // Reset announce timer
  wsUrl = "";                                                 // Clear any old WS URL (we’ll wait for a new one)
}

// ----- Send an UDP broadcast announce every ANNOUNCE_MS -----
void sendAnnounceIfDue()
{
  unsigned long now = millis();                               // Current time in ms
  if (now - lastAnnounce < ANNOUNCE_MS) return;               // If not time yet, skip
  lastAnnounce = now;                                         // Remember when we announced

  IPAddress broadcast(255, 255, 255, 255);                    // Broadcast IP (simple and common)

  // Build a tiny JSON-like text announcing our robotId (callsign left empty for now)
  String payload = String("{\"robotId\":\"") + robotId + "\",\"callsign\":\"\"}";

  udp.beginPacket(broadcast, DISCOVERY_PORT);                 // Begin a UDP packet to broadcast:DISCOVERY_PORT
  udp.write((const uint8_t*)payload.c_str(), payload.length()); // Put our text bytes in the packet
  udp.endPacket();                                            // Send it out

  Serial0.println("[DISCOVERY] Announce sent");               // Log we sent announce
}

// ----- Try to read one UDP reply and extract a "ws" URL -----
bool tryReadDiscoveryReply()
{
  int packetSize = udp.parsePacket();                         // Check if a UDP packet arrived
  if (packetSize <= 0) return false;                          // No packet → nothing to do

  char buf[256];                                              // Small buffer for incoming text
  int len = udp.read(buf, sizeof(buf) - 1);                   // Read packet bytes into buffer
  if (len <= 0) return false;                                 // If read failed, give up
  buf[len] = '\0';                                            // Null-terminate the C string

  String s = String(buf);                                     // Wrap into Arduino String for easy find
  int wsIdx = s.indexOf("\"ws\"");                            // Look for the key "ws"
  if (wsIdx < 0) return false;                                // Not found → ignore this packet
  int colon = s.indexOf(':', wsIdx);                          // Find the colon after "ws"
  if (colon < 0) return false;                                // If no colon, bad format
  int q1 = s.indexOf('"', colon + 1);                         // Find first quote after colon
  int q2 = s.indexOf('"', q1 + 1);                            // Find closing quote
  if (q1 < 0 || q2 < 0) return false;                         // If quotes missing, bad format

  wsUrl = s.substring(q1 + 1, q2);                            // Extract URL between the quotes
  Serial0.print("[DISCOVERY] Got WS URL: ");                  // Log the URL we found
  Serial0.println(wsUrl);                                     // Print the URL
  return true;                                                // Signal success
}

// ----- Connect WebSocket using the URL string (simple + proven) -----
void connectWebSocket()
{
  if (wsUrl.length() == 0)                                    // If we don’t have a URL yet…
  {
    Serial0.println("[WS] No URL yet");                       // …log and return
    return;                                                   // Exit
  }

  ws.close();                                                 // Ensure previous socket is closed (cleanup)
  delay(200);                                                 // Small pause to be safe

  // Set message handler: runs when text arrives from server
  ws.onMessage([](WebsocketsMessage msg)                      // Register a lambda to handle messages
  {
    if (!msg.isText()) return;                                // Ignore binary frames for now
    Serial0.print("[WS] RX: ");                               // Log a prefix
    Serial0.println(msg.data());                              // Print the received text
  });

  // Set event handler: runs on open/close/ping/pong
  ws.onEvent([](WebsocketsEvent e, String)                    // Register a lambda for events
  {
    if (e == WebsocketsEvent::ConnectionOpened)               // If connection just opened
    {
      Serial0.println("[WS] Opened");                         // Log it
    }
    else if (e == WebsocketsEvent::ConnectionClosed)          // If connection closed
    {
      Serial0.println("[WS] Closed -> restarting discovery"); // Log closure
      wsUrl = "";                                             // Clear URL so we go back to UDP
      startDiscovery();                                       // Reopen UDP and start announcing again
    }
    else if (e == WebsocketsEvent::GotPing)                   // If server pinged us
    {
      Serial0.println("[WS] Got ping");                       // Log ping
    }
    else if (e == WebsocketsEvent::GotPong)                   // If we got a pong
    {
      Serial0.println("[WS] Got pong");                       // Log pong
    }
  });

  Serial0.print("[WS] Connecting ");                          // Log we’re about to connect
  Serial0.println(wsUrl);                                     // Show the URL we will use
  
  while (!ws.connect("ws://192.168.86.197:8080/esp32"))
  //while (!ws.connect(wsUrl.c_str()))                          // Try to connect in a simple retry loop
  {
    Serial0.print(".");                                       // Dot = retry
    delay(300);                                               // Small wait between tries
  }
  Serial0.println("\n[WS] Connected");                        // Connected!

  // Immediately send a hello with our id so server can register us
  String hello = String("{\"cmd\":\"hello\",\"id\":\"") + robotId + "\"}";
  ws.send(hello);                                             // Send text frame to server

  lastHeartbeat = 0;                                          // Reset HB timer so we send soon
}

// ----- Send a heartbeat every HEARTBEAT_MS while WS is connected -----
void sendHeartbeatIfDue()
{
  unsigned long now = millis();                               // Current time in ms
  if (now - lastHeartbeat < HEARTBEAT_MS) return;             // Not time yet → skip
  lastHeartbeat = now;                                        // Update last HB time

  String hb = String("{\"cmd\":\"hb\",\"t\":") + String((unsigned long)millis()) + "}"; // Simple JSON
  ws.send(hb);                                                // Send HB to server
}

// ======== Arduino setup() runs once on boot ========
void setup()
{
  Serial0.begin(115200);                                      // Start USB-serial for logs (ESP32-S3 uses Serial0)
  delay(100);                                                 // Tiny delay to let serial settle

  Serial0.println("[SETUP] Starting ESP32 Robot Connector");  // Startup banner

  robotId = getRobotId();                                     // Create our unique ID from MAC
  Serial0.print("[ID] robotId=");                             // Log label
  Serial0.println(robotId);                                   // Log the ID

  connectWifi();                                              // Join the Wi-Fi network
  startDiscovery();                                           // Begin UDP announce loop
}

// ======== Arduino loop() runs forever ========
void loop()
{
  if (wsUrl.length() == 0)                                    // If we don’t have a WS URL yet…
  {
    sendAnnounceIfDue();                                      // Send a UDP announce if it’s time
    if (tryReadDiscoveryReply())                              // If we received a WS URL reply…
    {
      connectWebSocket();                                     // …connect to that WebSocket URL
    }
    delay(10);                                                // Short nap to be polite
    return;                                                   // Done for this loop tick
  }

  if (ws.available())                                         // If WebSocket is currently connected…
  {
    ws.poll();                                                // Process incoming messages/events
    sendHeartbeatIfDue();                                     // Send heartbeat periodically
  }
  else                                                        // If WebSocket is not connected…
  {
    Serial0.println("[WS] Not connected -> restarting discovery"); // Log that we’re restarting
    wsUrl = "";                                               // Clear URL to force discovery
    startDiscovery();                                         // Reopen UDP + start announcing again
  }

  delay(10);                                                  // Small delay to avoid busy looping
}
// ===== Minimal ESP32 Robot Connector (END) =====
