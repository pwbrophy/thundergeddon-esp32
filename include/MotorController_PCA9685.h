// MotorController_PCA9685.h
// Simple PCA9685 motor + turret controller for DRV8833 with OE (GPIO) and nSLEEP (via PCA ch14).
// Every line commented for clarity.

#pragma once                             // Prevent double-include

#include <Arduino.h>                     // Arduino core
#include <Wire.h>                        // I2C
#include <Adafruit_PWMServoDriver.h>     // PCA9685 driver (Adafruit)

// ------------ Your channel map (matches your old code) ------------
constexpr uint8_t L_IN1_CH = 10;         // Left motor: IN1 (A)
constexpr uint8_t L_IN2_CH = 11;         // Left motor: IN2 (B)
constexpr uint8_t R_IN1_CH = 0;          // Right motor: IN1 (A)
constexpr uint8_t R_IN2_CH = 1;          // Right motor: IN2 (B)
constexpr uint8_t T_IN1_CH = 8;          // Turret: IN1 (A)
constexpr uint8_t T_IN2_CH = 9;          // Turret: IN2 (B)

// ------------ DRV8833 nSLEEP ------------
constexpr uint8_t NSLEEP_CH = 14;        // DRV8833 nSLEEP is wired to PCA9685 channel 14 (active HIGH)

class MotorController_PCA9685 {
public:
  // Begin the driver.
  //  - i2c_addr : PCA9685 I2C address (0x40 default)
  //  - pwm_hz   : PWM frequency (e.g., 1000 Hz for DC motors)
  //  - sda/scl  : explicit I2C pins for ESP32 (do NOT use camera pins)
  //  - oePin    : GPIO that goes to PCA9685 OE pin (LOW = outputs enabled)
  bool begin(uint8_t i2c_addr, float pwm_hz, int sda, int scl, int oePin) {
    // Store config
    _addr  = i2c_addr;                   // Save the I2C address
    _pwmHz = pwm_hz;                     // Save the PWM frequency
    _oePin = oePin;                      // Save the OE GPIO number

    // Start I2C on the requested pins (or defaults if -1)
    if (sda >= 0 && scl >= 0) Wire.begin(sda, scl); else Wire.begin();  // Init I2C
    Wire.setClock(400000);               // Use 400kHz I2C for snappier updates

    // Configure and assert OE: LOW means PCA outputs are enabled
    if (_oePin >= 0) {                   // If an OE pin is provided
      pinMode(_oePin, OUTPUT);           // Make OE a push-pull output
      digitalWrite(_oePin, LOW);         // Drive LOW -> enable PCA outputs
      Serial0.printf("[MOT] OE(GPIO %d) = LOW (PCA outputs enabled)\n", _oePin);
    } else {
      Serial0.println("[MOT] No OE pin provided (assuming hardware-tied LOW).");
    }

    // Create and initialize the PCA9685 device
    _pwm = new Adafruit_PWMServoDriver(_addr);     // Allocate the driver object
    if (!_pwm) {                                   // Check allocation
      Serial0.println("[MOT] NEW Adafruit_PWMServoDriver failed"); 
      return false;                                 // Fail init
    }
    _pwm->begin();                                  // Wake the PCA9685
    _pwm->setPWMFreq(_pwmHz);                       // Set the PWM frequency
    Serial0.printf("[MOT] PCA9685 @0x%02X freq=%.1f Hz\n", _addr, _pwmHz);

    // Be safe at boot
    _enabled = false;                               // Software gate = off
    stopAll();                                      // All outputs = 0% duty

    // Ensure DRV8833 is asleep at boot (nSLEEP LOW)
    _setNSleep(false);                              // Keep bridges off until 'enable(true)'

    return true;                                     // Init OK
  }

  // Software enable/disable for motors.
  // Also toggles DRV8833 nSLEEP via the PCA channel.
  void enable(bool on) {
    _enabled = on;                                   // Record the new state
    Serial0.printf("[MOT] enable(%s)\n", on ? "true" : "false");
    if (on) {
      _setNSleep(true);                              // nSLEEP HIGH -> wake DRV8833
    } else {
      stopAll();                                     // Stop PWM on INx
      _setNSleep(false);                             // nSLEEP LOW  -> sleep DRV8833
    }
  }

  // Report enable state for debugging.
  bool isEnabled() const { return _enabled; }        // Return the software gate

  // Emergency stop (all INx = 0% duty). Leaves nSLEEP state untouched.
  void stopAll() {
    _writeDuty(L_IN1_CH, 0);                         // Left A  = 0%
    _writeDuty(L_IN2_CH, 0);                         // Left B  = 0%
    _writeDuty(R_IN1_CH, 0);                         // Right A = 0%
    _writeDuty(R_IN2_CH, 0);                         // Right B = 0%
    _writeDuty(T_IN1_CH, 0);                         // Turret A= 0%
    _writeDuty(T_IN2_CH, 0);                         // Turret B= 0%
    Serial0.println("[MOT] stopAll()");
  }

  // Drive left and right motors with signed values in [-1..+1]
  // +ve -> A gets duty, B=0 (forward). -ve -> A=0, B gets duty (reverse).
  void setLeftRight(float l, float r) {
    if (!_enabled) {                                 // If not enabled,
      Serial0.println("[MOT] setLeftRight ignored (enabled=false)"); 
      stopAll();                                     // ensure outputs are idle
      return;                                        // and bail
    }
    _drivePair(l, L_IN1_CH, L_IN2_CH);               // Drive the left H-bridge
    _drivePair(r, R_IN1_CH, R_IN2_CH);               // Drive the right H-bridge
  }

  // Drive turret with signed value in [-1..+1] (same A/B pattern).
  void setTurret(float v) {
    if (!_enabled) {                                 // If not enabled,
      Serial0.println("[MOT] setTurret ignored (enabled=false)");
      _writeDuty(T_IN1_CH, 0);                       // force off
      _writeDuty(T_IN2_CH, 0);                       // force off
      return;                                        // and bail
    }
    _drivePair(v, T_IN1_CH, T_IN2_CH);               // Drive the turret H-bridge
  }

private:
  // Set DRV8833 nSLEEP line through the PCA9685 channel.
  // true  -> HIGH (awake). false -> LOW (sleep).
  void _setNSleep(bool awake) {
    if (!_pwm) return;                               // Guard if driver missing
    // Adafruit API: setPin(channel, value) where value = 0..4095 (duty), 4096 = FULL ON.
    // We want a static HIGH or LOW, not PWM.
    if (awake) {
      _pwm->setPin(NSLEEP_CH, 4096);                // FULL ON -> logic HIGH on nSLEEP
      Serial0.printf("[MOT] nSLEEP ch%d = HIGH (awake)\n", NSLEEP_CH);
    } else {
      _pwm->setPin(NSLEEP_CH, 0);                   // FULL OFF -> logic LOW on nSLEEP
      Serial0.printf("[MOT] nSLEEP ch%d = LOW (sleep)\n", NSLEEP_CH);
    }
  }

  // Convert magnitude [0..1] to PCA9685 12-bit duty [0..4095]
  static uint16_t _toDuty(float mag) {
    mag = fminf(fmaxf(mag, 0.0f), 1.0f);            // Clamp to 0..1
    return (uint16_t)lroundf(mag * 4095.0f);        // Scale to 12-bit duty
  }

  // Write a raw duty to a specific PCA channel.
  void _writeDuty(uint8_t ch, uint16_t duty) {
    if (!_pwm) return;                               // Guard if driver is missing
    _pwm->setPWM(ch, 0, duty);                       // Simple duty: on=0, off=duty
  }

  // Drive one DRV8833 H-bridge pair using signed input [-1..+1]
  void _drivePair(float v, uint8_t chA, uint8_t chB) {
    v = fminf(fmaxf(v, -1.0f), 1.0f);                // Clamp input to -1..+1
    float mag = fabsf(v);                             // Magnitude
    if (mag < 0.001f) {                               // Tiny -> treat as zero
      _writeDuty(chA, 0);                             // A off
      _writeDuty(chB, 0);                             // B off
      return;                                         // Done
    }
    uint16_t duty = _toDuty(mag);                     // Convert to duty
    if (v > 0.0f) {                                   // Forward
      _writeDuty(chA, duty);                          // A = duty
      _writeDuty(chB, 0);                             // B = 0
    } else {                                          // Reverse
      _writeDuty(chA, 0);                             // A = 0
      _writeDuty(chB, duty);                          // B = duty
    }
  }

  // ------------- members -------------
  Adafruit_PWMServoDriver* _pwm = nullptr;           // PCA driver object
  uint8_t  _addr   = 0x40;                           // I2C address
  float    _pwmHz  = 1000.0f;                        // PWM frequency
  int      _oePin  = -1;                             // OE GPIO (LOW = enable PCA outputs)
  bool     _enabled = false;                         // Software enable flag
};
