// OtaSupport.h — ArduinoOTA wrapper with pause/resume hooks and reliability tweaks
#pragma once
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

namespace OtaSupport {
  // When true, your loop() should skip normal work
  static volatile bool active = false;

  using PauseFn  = void (*)();
  using ResumeFn = void (*)();

  static PauseFn  onPause  = nullptr;
  static ResumeFn onResume = nullptr;

  inline void begin(const char* hostname,
                    const char* password = nullptr,
                    PauseFn pauseFn = nullptr,
                    ResumeFn resumeFn = nullptr)
  {
    onPause  = pauseFn;
    onResume = resumeFn;

    // Wi-Fi stability for OTA
    WiFi.setSleep(false);                    // keep radio fully awake
    WiFi.setTxPower(WIFI_POWER_19_5dBm);     // strong signal (optional)

    if (hostname && *hostname) ArduinoOTA.setHostname(hostname);
    ArduinoOTA.setPort(3232);

    #ifdef OTA_PASSWORD
      const char* pw = (password && *password) ? password : OTA_PASSWORD;
    #else
      const char* pw = password;
    #endif
    if (pw && *pw) ArduinoOTA.setPassword(pw);

    ArduinoOTA.onStart([]() {
      active = true;
      if (onPause) onPause();                // let main close sockets/stop polling
      Serial0.println("[OTA] Start");
    });
    ArduinoOTA.onEnd([]() {
      Serial0.println("[OTA] End");
      // Usually reboots; no resume needed here
    });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
      static uint8_t last = 255;
      uint8_t pct = t ? (p * 100 / t) : 0;
      if (pct != last) { last = pct; Serial0.printf("[OTA] %u%%\n", pct); }
      yield();                                // keep Wi-Fi fed
    });
    ArduinoOTA.onError([](ota_error_t e) {
      Serial0.printf("[OTA] Error %u\n", e);
    });

    ArduinoOTA.begin();
    Serial0.println("[OTA] Ready (port 3232)");
  }

  inline void handle() { ArduinoOTA.handle(); }
}
