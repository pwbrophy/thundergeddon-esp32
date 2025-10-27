/*
  Minimal ESP32 robot connector (prototype)

  1) Connects to Wi-Fi (from secrets.h).
  2) Every 2 seconds, sends a UDP "announce" broadcast on port 30560 with:
       { "robotId": "<MAC>", "callsign": "" }
  3) Waits for a unicast UDP reply:
       { "ws": "ws://<server-ip>:8080/esp32" }
  4) Connects to that WebSocket and sends:
       { "cmd":"hello", "id":"<MAC>" }
  5) Sends heartbeat every 2 seconds:
       { "cmd":"hb", "t": 123456 }
  6) If the WS closes or times out, goes back to discovery.

  Keep it simple: no motors, no camera, no rename yet.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoWebsockets.h>
#include "secrets.h"   // define WIFI_SSID, WIFI_PASS

using namespace websockets;

// ---------------------- Config ----------------------
static const uint16_t DISCOVERY_PORT = 30560;       // UDP port for discovery
static const uint16_t WS_PORT        = 8080;        // WebSocket server port (Unity side)
static const char*    WS_PATH        = "/esp32";    // WebSocket path on the server
static const uint32_t ANNOUNCE_MS    = 2000;        // How often we broadcast announce
static const uint32_t HEARTBEAT_MS   = 2000;        // How often we send WS heartbeat
// ---------------------------------------------------

WiFiUDP udp;
WebsocketsClient ws;

String robotId;         // stable id based on MAC
String wsUrl;           // server-provided WebSocket URL, e.g. "ws://192.168.0.10:8080/esp32"

unsigned long lastAnnounce = 0;
unsigned long lastHeartbeat = 0;

String getRobotId()
{
  uint64_t mac = ESP.getEfuseMac();
  char buf[13];
  snprintf(buf, sizeof(buf),
           "%02X%02X%02X%02X%02X%02X",
           (uint8_t)(mac >> 40),
           (uint8_t)(mac >> 32),
           (uint8_t)(mac >> 24),
           (uint8_t)(mac >> 16),
           (uint8_t)(mac >> 8),
           (uint8_t)mac);
  return String(buf);
}

// Very small helper: try to read a UDP reply into wsUrl.
// Returns true if a valid "ws" field was parsed and stored.
bool tryReadDiscoveryReply()
{
  int packetSize = udp.parsePacket();
  if (packetSize <= 0)
  {
    return false;
  }

  // Read packet data into a buffer
  char buf[256];
  int len = udp.read(buf, sizeof(buf) - 1);
  if (len <= 0)
  {
    return false;
  }
  buf[len] = '\0';

  // Very naive parse: look for "ws":"...".
  // (We will keep it simple to avoid adding a JSON lib.)
  String s = String(buf);
  int wsIdx = s.indexOf("\"ws\"");
  if (wsIdx < 0)
  {
    return false;
  }
  int colon = s.indexOf(':', wsIdx);
  if (colon < 0)
  {
    return false;
  }
  int q1 = s.indexOf('"', colon + 1);
  int q2 = s.indexOf('"', q1 + 1);
  if (q1 < 0 || q2 < 0)
  {
    return false;
  }
  wsUrl = s.substring(q1 + 1, q2);
  Serial0.print("[DISCOVERY] Got WS URL: ");
  Serial0.println(wsUrl);
  return true;
}

void connectWifi()
{
  Serial0.print("[WIFI] Connecting to ");
  Serial0.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial0.print(".");
    delay(300);
  }

  Serial0.print("\n[WIFI] IP: ");
  Serial0.println(WiFi.localIP());
}

void startDiscovery()
{
  Serial0.println("[DISCOVERY] Starting UDP announce loop");

  // Begin UDP. Local port can be any; we just need to be able to receive replies.
  udp.begin(DISCOVERY_PORT);
  lastAnnounce = 0;
  wsUrl = "";
}

void sendAnnounceIfDue()
{
  unsigned long now = millis();
  if (now - lastAnnounce < ANNOUNCE_MS)
  {
    return;
  }
  lastAnnounce = now;

  // Broadcast address
  IPAddress broadcast(255, 255, 255, 255);

  // Minimal JSON announce
  // You can include IP if you want, but server reads sender IP anyway.
  String payload = String("{\"robotId\":\"") + robotId + "\",\"callsign\":\"\"}";

  // Send packet
  udp.beginPacket(broadcast, DISCOVERY_PORT);
  udp.write((const uint8_t*)payload.c_str(), payload.length());
  udp.endPacket();

  Serial0.println("[DISCOVERY] Announce sent");
}

void closeWebSocket()
{
  if (ws.available())
  {
    ws.close();
  }
}

bool connectWebSocket()
{
  if (wsUrl.length() == 0)
  {
    return false;
  }

  // Make sure client is clean
  closeWebSocket();
  delay(100);

  // Simple event handlers
  ws.onMessage([](WebsocketsMessage msg)
  {
    if (!msg.isText())
    {
      return;
    }
    // For prototype we only log incoming text
    Serial0.print("[WS] RX: ");
    Serial0.println(msg.data());
  });

  ws.onEvent([](WebsocketsEvent e, String data)
  {
    if (e == WebsocketsEvent::ConnectionOpened)
    {
      Serial0.println("[WS] Opened");
    }
    else if (e == WebsocketsEvent::ConnectionClosed)
    {
      Serial0.println("[WS] Closed");
    }
    else if (e == WebsocketsEvent::GotPing)
    {
      Serial0.println("[WS] Got ping");
    }
    else if (e == WebsocketsEvent::GotPong)
    {
      Serial0.println("[WS] Got pong");
    }
  });

  Serial0.print("[WS] Connecting to ");
  Serial0.println(wsUrl);

  // Connect (blocking retry for simplicity)
  while (!ws.connect(wsUrl.c_str()))
  {
    Serial0.print(".");
    delay(300);
  }
  Serial0.println("\n[WS] Connected");

  // Send a hello with our id
  String hello = String("{\"cmd\":\"hello\",\"id\":\"") + robotId + "\"}";
  ws.send(hello);

  // Reset heartbeat timer so we start sending soon
  lastHeartbeat = 0;

  return true;
}

void sendHeartbeatIfDue()
{
  unsigned long now = millis();
  if (now - lastHeartbeat < HEARTBEAT_MS)
  {
    return;
  }
  lastHeartbeat = now;

  // Minimal heartbeat message with a timestamp
  String hb = String("{\"cmd\":\"hb\",\"t\":") + String((unsigned long)millis()) + "}";
  ws.send(hb);
}

void setup()
{
  Serial0.begin(115200);
  delay(3000);

  Serial0.println("\n[SETUP] Starting ESP32 Robot Connector");
  delay(500);
  robotId = getRobotId();
  Serial0.print("[ID] robotId=");
  Serial0.println(robotId);

  connectWifi();
  startDiscovery();
}

void loop()
{
  // If we do not have a WS URL yet, keep announcing and try to read a reply
  if (wsUrl.length() == 0)
  {
    sendAnnounceIfDue();
    if (tryReadDiscoveryReply())
    {
      // We got a WS URL; attempt to connect
      connectWebSocket();
    }
    delay(10);
    return;
  }

  // If we have a WS URL, keep the websocket alive
  if (ws.available())
  {
    ws.poll();                   // process incoming and events
    sendHeartbeatIfDue();        // send hb
  }
  else
  {
    // WS disconnected → forget URL and restart discovery
    Serial0.println("[WS] Not connected → restarting discovery");
    wsUrl = "";
    startDiscovery();
  }

  delay(10);
}
