// OtaSupport.h
#pragma once
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

namespace OtaSupport {
  // Expose a flag so main.cpp can skip work during OTA
  static volatile bool active = false;

  // Optional: give callers a way to pause/resume their app
  typedef void (*PauseFn)();
  typedef void (*ResumeFn)();
  static PauseFn onPause = nullptr;
  static ResumeFn onResume = nullptr;

  inline void begin(const char* hostname, const char* password = nullptr,
                    PauseFn pauseFn = nullptr, ResumeFn resumeFn = nullptr) {
    onPause  = pauseFn;
    onResume = resumeFn;

    // Wi-Fi stability tweaks
    WiFi.setSleep(false);                           // prevent modem sleep during OTA
    WiFi.setTxPower(WIFI_POWER_19_5dBm);            // max TX power (optional)

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
      if (onPause) onPause();                       // let app stop sockets/timers
      Serial0.println("[OTA] Start");
    });
    ArduinoOTA.onEnd([]() {
      Serial0.println("[OTA] End");
      // active will go false after reboot; no resume needed here
    });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
      static uint8_t last = 255;
      uint8_t pct = t ? (p * 100 / t) : 0;
      if (pct != last) { last = pct; Serial0.printf("[OTA] %u%%\n", pct); }
      yield();                                      // keep Wi-Fi fed
    });
    ArduinoOTA.onError([](ota_error_t e) {
      Serial0.printf("[OTA] Error %u\n", e);
    });

    ArduinoOTA.begin();
    Serial0.println("[OTA] Ready (port 3232)");
  }

  inline void handle() { ArduinoOTA.handle(); }
}
