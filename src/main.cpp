// ===== ESP32-S3 Robot Client: Wi-Fi + UDP discovery + WebSocket + OTA =====

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include "secrets.h"        // #define WIFI_SSID "..."  #define WIFI_PASS "..."
#include "LedController.h"  // your non-blocking LED helper
#include "OtaSupport.h"     // header-only OTA wrapper

using namespace websockets;

// ---- Tunables ----
static const uint16_t DISCOVERY_PORT = 30560;
static const uint32_t ANNOUNCE_MS    = 2000;
static const uint32_t HEARTBEAT_MS   = 2000;

// ---- State ----
WiFiUDP udp;
WebsocketsClient ws;
LedController led;

String robotId;
String wsUrl;
unsigned long lastAnnounce = 0;
unsigned long lastHeartbeat = 0;

// ----- Helpers -----
String getRobotId() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[13];
  snprintf(buf, sizeof(buf),
           "%02X%02X%02X%02X%02X%02X",
           (uint8_t)(mac >> 40), (uint8_t)(mac >> 32), (uint8_t)(mac >> 24),
           (uint8_t)(mac >> 16), (uint8_t)(mac >> 8),  (uint8_t)(mac >> 0));
  return String(buf);
}

void connectWifi() {
  Serial0.printf("[WiFi] Connecting to '%s' ...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial0.println("[WiFi] Failed, retrying in 2s");
    delay(2000);
  }
  Serial0.printf("[WiFi] Connected. IP=%s RSSI=%d\n",
                 WiFi.localIP().toString().c_str(), WiFi.RSSI());
}

void startDiscovery() {
  udp.stop();
  if (!udp.begin(DISCOVERY_PORT)) {
    Serial0.println("[UDP] Failed to bind discovery port");
  } else {
    Serial0.printf("[UDP] Listening on %u\n", DISCOVERY_PORT);
  }
  lastAnnounce = 0;
}

void sendAnnounceIfDue() {
  unsigned long now = millis();
  if (now - lastAnnounce < ANNOUNCE_MS) return;
  lastAnnounce = now;

  // Minimal JSON announce (string-built for speed)
  String payload = String("{\"robotId\":\"") + robotId + "\",\"callsign\":\"\"}";

  udp.beginPacket(IPAddress(255,255,255,255), DISCOVERY_PORT);
  udp.write((const uint8_t*)payload.c_str(), payload.length());
  udp.endPacket();

  Serial0.println("[UDP] Announce sent");
}

bool tryReadDiscoveryReply() {
  int sz = udp.parsePacket();
  if (sz <= 0) return false;

  char buf[512];
  int n = udp.read((uint8_t*)buf, sizeof(buf) - 1);
  if (n <= 0) return false;
  buf[n] = 0;

  // Expect {"ws":"ws://IP:PORT/path"}
  String s(buf);
  int i = s.indexOf("\"ws\"");
  if (i < 0) return false;
  int colon = s.indexOf(':', i);
  if (colon < 0) return false;
  int q1 = s.indexOf('"', colon + 1);
  if (q1 < 0) return false;
  int q2 = s.indexOf('"', q1 + 1);
  if (q2 < 0) return false;

  wsUrl = s.substring(q1 + 1, q2);
  Serial0.printf("[DISCOVERY] Got ws URL: %s\n", wsUrl.c_str());
  return true;
}

void connectWebSocket() {
  if (wsUrl.isEmpty()) {
    Serial0.println("[WS] No URL yet");
    return;
  }
  ws.close();
  delay(200);

  ws.onMessage([](WebsocketsMessage msg) {
    if (!msg.isText()) return;
    String s = msg.data();
    Serial0.printf("[WS] RX: %s\n", s.c_str());

    // Example: {"cmd":"flash","pin":48,"ms":2000}
    if (s.indexOf("\"cmd\":\"flash\"") >= 0) {
      int pinVal = 48, msVal = 2000;
      int pIdx = s.indexOf("\"pin\"");
      if (pIdx >= 0) { int c = s.indexOf(':', pIdx); if (c >= 0) pinVal = s.substring(c + 1).toInt(); }
      int mIdx = s.indexOf("\"ms\"");
      if (mIdx >= 0) { int c = s.indexOf(':', mIdx); if (c >= 0) msVal  = s.substring(c + 1).toInt(); }
      if (pinVal == 48) led.startFlash((uint32_t)msVal);
    }
  });

  ws.onEvent([](WebsocketsEvent e, String) {
    if (e == WebsocketsEvent::ConnectionOpened) {
      Serial0.println("[WS] Opened");
    } else if (e == WebsocketsEvent::ConnectionClosed) {
      Serial0.println("[WS] Closed -> rediscover");
      wsUrl = "";
      startDiscovery();
    } else if (e == WebsocketsEvent::GotPing) {
      Serial0.println("[WS] Ping");
    } else if (e == WebsocketsEvent::GotPong) {
      Serial0.println("[WS] Pong");
    }
  });

  Serial0.printf("[WS] Connecting %s\n", wsUrl.c_str());
  while (!ws.connect(wsUrl.c_str())) {
    Serial0.print("."); delay(300);
  }
  Serial0.println("\n[WS] Connected");

  String hello = String("{\"cmd\":\"hello\",\"id\":\"") + robotId + "\"}";
  ws.send(hello);
  lastHeartbeat = 0;
}

void sendHeartbeatIfDue() {
  unsigned long now = millis();
  if (now - lastHeartbeat < HEARTBEAT_MS) return;
  lastHeartbeat = now;

  String hb = String("{\"cmd\":\"hb\",\"t\":") + String((unsigned long)millis()) + "}";
  ws.send(hb);
}

// ---- OTA pause/resume hooks ----
void pauseForOta() {
  udp.stop();      // stop discovery sockets
  ws.close();      // stop WS traffic
}
void resumeAfterOta() {
  // usually reset happens; this is here for completeness
  udp.begin(DISCOVERY_PORT);
  lastAnnounce = 0;
}

// ==== Arduino setup/loop ====
void setup() {
  Serial0.begin(115200);
  delay(100);
  Serial0.println("[BOOT] Starting");

  led.begin(48);                 // WS2812 on GPIO 48 (adjust to your pin)

  robotId = getRobotId();
  Serial0.printf("[ID] %s\n", robotId.c_str());

  connectWifi();

  // Start OTA (hostname=robotId, password from build flag OTA_PASSWORD)
  OtaSupport::begin(robotId.c_str(), nullptr, pauseForOta, resumeAfterOta);

  startDiscovery();
}

void loop() {
  // 1) Service OTA first; skip all else if active
  OtaSupport::handle();
  if (OtaSupport::active) { delay(5); return; }

  // 2) Discovery / connect
  if (wsUrl.isEmpty()) {
    sendAnnounceIfDue();
    if (tryReadDiscoveryReply()) connectWebSocket();
    delay(10);
    return;
  }

  // 3) WS poll + heartbeat or restart discovery on drop
  if (ws.available()) {
    ws.poll();
    sendHeartbeatIfDue();
  } else {
    Serial0.println("[WS] Not connected -> rediscover");
    wsUrl = "";
    startDiscovery();
  }

  // 4) LEDs & small yield
  led.update(millis());
  delay(10);
}
