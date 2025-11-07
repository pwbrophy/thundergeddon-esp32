// CameraController.h — non-blocking ESP32-S3 OV camera wrapper (header-only for simplicity)
#pragma once                                      // Ensure this header is compiled once

#include <Arduino.h>                              // Arduino core (millis, types, etc.)
#include "esp_camera.h"                           // ESP32 camera driver (JPEG capture)

// This class owns camera init/start/stop and one-shot JPEG capture.
// It keeps the camera OFF by default so OTA / WiFi connect aren’t impacted.
class CameraController
{
public:                                           // Public API used by main.cpp

  // Constructor: set defaults (but do NOT touch hardware yet).
  CameraController()
  : _started(false)                               // Camera starts disabled at boot
  , _frameSize(FRAMESIZE_VGA)                   // Default to QVGA (320x240) for modest bandwidth
  , _jpegQuality(12)                              // JPEG quality (lower = better quality, larger bytes)
  {}

  // Configure pin map + base settings, but do not power sensor yet.
  // Call this once early in setup().
  void begin()                                    // No arguments; we hardcode the S3 WROOM N16R8 pinout
  {
    // Zero the config so all fields are predictable.
    memset(&_cfg, 0, sizeof(_cfg));               // Clear the camera_config_t struct

    // ---- Mandatory pins for ESP32-S3 parallel camera ----
    _cfg.ledc_channel = LEDC_CHANNEL_0;           // Use LEDC channel 0 for XCLK
    _cfg.ledc_timer   = LEDC_TIMER_0;             // Use LEDC timer 0 for XCLK

    _cfg.pin_pwdn  = -1;                          // Power-down pin not used on many modules
    _cfg.pin_reset = -1;                          // Reset pin not wired (use -1)

    _cfg.pin_xclk  = 15;                          // XCLK pin:  GPIO15 (per your table)
    _cfg.pin_sccb_sda = 4;                        // SIOD pin:  GPIO04
    _cfg.pin_sccb_scl = 5;                        // SIOC pin:  GPIO05

    _cfg.pin_d7 = 18;                              // Y7  -> GPIO18
    _cfg.pin_d6 = 17;                              // Y8  -> GPIO17
    _cfg.pin_d5 = 16;                              // Y9  -> GPIO16
    _cfg.pin_d4 = 8;                               // Y4  -> GPIO08
    _cfg.pin_d3 = 9;                               // Y3  -> GPIO09
    _cfg.pin_d2 = 11;                              // Y2  -> GPIO11
    _cfg.pin_d1 = 12;                              // Y6  -> GPIO12
    _cfg.pin_d0 = 10;                              // Y5  -> GPIO10

    _cfg.pin_vsync = 6;                            // VSYNC -> GPIO06
    _cfg.pin_href  = 7;                            // HREF  -> GPIO07
    _cfg.pin_pclk  = 13;                           // PCLK  -> GPIO13

    // ---- Timing / format ----
    _cfg.xclk_freq_hz = 15000000;                 // XCLK at 15 MHz (stable choice for S3 + PSRAM)
    _cfg.pixel_format = PIXFORMAT_JPEG;           // We want compressed JPEG frames

    // ---- Frame buffer settings ----
    _cfg.frame_size   = _frameSize;               // Initial frame size (QVGA by default)
    _cfg.jpeg_quality = _jpegQuality;             // Quality setting 10–12 is a good start on S3
    _cfg.fb_count     = 2;                        // Two frame buffers improves throughput
    _cfg.fb_location  = CAMERA_FB_IN_PSRAM;       // Use external PSRAM (S3 WROOM has 8MB)
    _cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;   // Don’t block if no buffer yet

    // NOTE: We delay actual camera_init() until start() to keep boot light.
  }

  // Start the sensor (idempotent). Returns true on success.
  bool start()
  {
    if (_started) return true;                    // Already running → OK

    esp_err_t err = esp_camera_init(&_cfg);       // Power up + init camera driver
    if (err != ESP_OK)                            // Check result from the driver
    {
      // If init fails, try once more with a smaller frame to reduce memory pressure.
      _cfg.frame_size = FRAMESIZE_QVGA;         // 160x120 fallback
      err = esp_camera_init(&_cfg);               // Retry init
      if (err != ESP_OK)                          // Still failed?
      {
        return false;                             // Give up; caller can log and retry later
      }
    }

    // Apply sensor-specific tweaks (optional; safe if sensor null).
    if (sensor_t* s = esp_camera_sensor_get())    // Get pointer to active sensor
    {
      s->set_brightness(s, 0);                    // Neutral brightness
      s->set_saturation(s, 0);                    // Neutral saturation
      s->set_vflip(s, 1);                         // Flip vertically if needed for your mount (1=on, 0=off)
      s->set_hmirror(s, 1);                       // Mirror horizontally for front-facing view (1=on)
    }

    _started = true;                              // Record running state
    return true;                                  // Signal success
  }

  // Stop the sensor and free camera driver (idempotent).
  void stop()
  {
    if (!_started) return;                        // Already stopped → nothing to do
    esp_camera_deinit();                          // Power down / release driver & buffers
    _started = false;                             // Record stopped state
  }

  // Quick query for current state.
  bool isStarted() const
  {
    return _started;                              // Return cached flag
  }

  // Capture one JPEG frame. Returns pointer/length OR nullptr on failure.
  // The caller MUST return the buffer with esp_camera_fb_return().
  camera_fb_t* capture()
  {
    if (!_started) return nullptr;                // Not started → cannot capture
    camera_fb_t* fb = esp_camera_fb_get();       // Acquire a filled frame buffer
    if (!fb) return nullptr;                      // If allocation/capture failed → nullptr
    if (fb->format != PIXFORMAT_JPEG)            // If sensor delivered non-JPEG (rare with PIXFORMAT_JPEG)
    {
      esp_camera_fb_return(fb);                  // Return buffer immediately
      return nullptr;                            // Tell caller we don’t have a JPEG
    }
    return fb;                                   // Hand buffer to caller
  }

private:                                          // Internals hidden from users
  camera_config_t _cfg;                           // Camera driver configuration
  bool            _started;                       // Whether the camera is running
  framesize_t     _frameSize;                     // Desired frame size (we start at QVGA)
  int             _jpegQuality;                   // JPEG quality (lower -> higher quality)
};
