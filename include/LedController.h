// LedController.h — tiny non-blocking LED helper for one WS2812 on a chosen pin
#pragma once                                // Ensure this header is only included once

#include <Adafruit_NeoPixel.h>              // Library to drive WS2812/NeoPixel LEDs
#include <Arduino.h>                        // For Arduino types like uint8_t, millis()

class LedController                        // Small class to own LED setup + effects
{
public:                                     // Public methods callable by main.cpp
  LedController()                           // Default constructor (does nothing yet)
  : _pin(48),                               // Default GPIO pin for the LED (48)
    _count(1),                              // We control exactly one LED
    _strip(_count, _pin, NEO_GRB + NEO_KHZ800), // Create the NeoPixel strip driver
    _active(false),                         // No effect running at start
    _endAt(0),                              // End time placeholder
    _nextToggle(0),                         // Next toggle time placeholder
    _greenPhase(true)                       // Start with green on first toggle
  {}

  void begin(int pin = 48)                  // Initialize hardware; optional custom pin
  {
    _pin = pin;                             // Store the chosen GPIO pin
    _strip.setPin(_pin);                    // Tell the strip which pin to use
    _strip.begin();                         // Initialize the NeoPixel driver
    _strip.clear();                         // Ensure the LED is off initially
    _strip.show();                          // Push the off state to the LED
  }

  void startFlash(uint32_t durationMs)      // Begin a green/white flash for durationMs
  {
    _active = true;                         // Mark the effect as active
    _endAt = millis() + durationMs;         // Compute when the effect should stop
    _nextToggle = 0;                        // Force an immediate toggle on first update
    _greenPhase = true;                     // Start with green
  }

  void stop()                               // Stop any effect and turn the LED off
  {
    _active = false;                        // Mark effect as inactive
    _strip.clear();                         // Clear LED to black
    _strip.show();                          // Push the off state to hardware
  }

  void update(uint32_t nowMs)               // Advance the effect without blocking
  {
    if (!_active) return;                   // If nothing is running, do nothing

    if ((int32_t)(_endAt - nowMs) <= 0)     // If the effect has reached its end time
    {
      stop();                               // Turn off and finish
      return;                               // Exit early
    }

    if ((int32_t)(_nextToggle - nowMs) <= 0) // If it's time to toggle the color
    {
      if (_greenPhase)                      // If we are in the green phase
      {
        _showRGB(0, 255, 0);                // Show green (R=0,G=255,B=0)
      }
      else                                  // Otherwise we are in the white phase
      {
        _showRGB(255, 255, 255);            // Show white (R=255,G=255,B=255)
      }

      _greenPhase = !_greenPhase;           // Flip phase for next time
      _nextToggle = nowMs + 200;            // Schedule next toggle in ~200 ms
    }
  }

  bool isActive() const                     // Query whether an effect is running
  {
    return _active;                         // Return current active flag
  }

private:                                    // Private helpers and state
  void _showRGB(uint8_t r, uint8_t g, uint8_t b) // Set LED color immediately
  {
    _strip.setPixelColor(0, _strip.Color(r, g, b)); // Put RGB into pixel index 0
    _strip.show();                           // Push the color to the LED
  }

  int _pin;                                  // GPIO pin number used for the LED
  int _count;                                // Number of LEDs (always 1 here)
  Adafruit_NeoPixel _strip;                  // NeoPixel driver instance

  bool _active;                              // Whether a flashing effect is running
  uint32_t _endAt;                           // Absolute time (millis) when effect ends
  uint32_t _nextToggle;                      // Absolute time (millis) for next color flip
  bool _greenPhase;                          // true=green, false=white
};
