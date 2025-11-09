#pragma once

#include <Arduino.h>
#include "esp_camera.h"

class CameraController
{
public:
  CameraController()
  : _started(false),
    _frameSize(FRAMESIZE_VGA),   // VGA (640x480)
    _jpegQuality(10)             // close to your old working setting
  {
    memset(&_cfg, 0, sizeof(_cfg));
  }

  // Configure pins and static options (call once at boot)
  void begin()
  {
    // SCCB
    _cfg.pin_sccb_sda  = 4;    // SIOD -> GPIO04
    _cfg.pin_sccb_scl  = 5;    // SIOC -> GPIO05

    // Sync/clock
    _cfg.pin_vsync     = 6;    // VSYNC -> GPIO06
    _cfg.pin_href      = 7;    // HREF  -> GPIO07
    _cfg.pin_pclk      = 13;   // PCLK  -> GPIO13
    _cfg.pin_xclk      = 15;   // XCLK  -> GPIO15

    // Data lines (D0..D7 must be Y2..Y9 in order)
    _cfg.pin_d0        = 11;   // D0 = Y2 -> GPIO11
    _cfg.pin_d1        = 9;    // D1 = Y3 -> GPIO09
    _cfg.pin_d2        = 8;    // D2 = Y4 -> GPIO08
    _cfg.pin_d3        = 10;   // D3 = Y5 -> GPIO10
    _cfg.pin_d4        = 12;   // D4 = Y6 -> GPIO12
    _cfg.pin_d5        = 18;   // D5 = Y7 -> GPIO18
    _cfg.pin_d6        = 17;   // D6 = Y8 -> GPIO17
    _cfg.pin_d7        = 16;   // D7 = Y9 -> GPIO16

    // No PWDN/RESET on this module
    _cfg.pin_pwdn  = -1;
    _cfg.pin_reset = -1;

    // Clock + output format
    _cfg.ledc_channel = LEDC_CHANNEL_0;
    _cfg.ledc_timer   = LEDC_TIMER_0;
    _cfg.xclk_freq_hz = 22000000;           // 22 MHz (what your old file used)
    _cfg.pixel_format = PIXFORMAT_JPEG;     // we stream JPEG

    // Buffers + quality
    _cfg.frame_size   = _frameSize;         // VGA
    _cfg.jpeg_quality = _jpegQuality;       // ~10
    _cfg.fb_count     = 2;                  // 2 buffers
    _cfg.fb_location  = CAMERA_FB_IN_PSRAM; // use PSRAM
    _cfg.grab_mode    = CAMERA_GRAB_LATEST; // always return the newest
  }

  // Power up / init camera driver (idempotent)
  bool start()
  {
    if (_started) {
      Serial0.println("[CAM] start(): already started");
      return true;
    }

    Serial0.println("[CAM] start(): init driver…");
    esp_err_t err = esp_camera_init(&_cfg);
    if (err != ESP_OK) {
      Serial0.printf("[CAM] init failed (%d)\n", (int)err);
      return false;
    }

    if (sensor_t* s = esp_camera_sensor_get())
    {
      // Force runtime settings to make sure sensor obeys
      s->set_framesize(s, _frameSize);
      s->set_quality(s, _jpegQuality);
      s->set_brightness(s, 0);
      s->set_saturation(s, 0);
      s->set_vflip(s, 1);
      s->set_hmirror(s, 1);
    }

    _started = true;
    Serial0.println("[CAM] start(): OK");
    return true;
  }

  // Power down (idempotent)
  void stop()
  {
    if (!_started) return;
    esp_camera_deinit();
    _started = false;
    Serial0.println("[CAM] stop(): deinitialized");
  }

  bool isStarted() const { return _started; }

private:
  camera_config_t  _cfg;
  bool             _started;
  framesize_t      _frameSize;
  int              _jpegQuality;
};
