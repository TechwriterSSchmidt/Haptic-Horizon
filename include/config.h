#pragma once
#include <Arduino.h>

// ===== Pin Definitions (SuperMini NRF52840 / Nice!Nano) =====
// We use the direct GPIO numbers (P0.xx) because the board variant 
// might map "D1" to something else.
// P0.06 = 6, P0.17 = 17, P0.20 = 20
// Note: On Adafruit nRF52 core, we can often use the pin number directly 
// if we are careful, or use the P0_06 macros if defined.
// For safety, we define them as integers matching the GPIO port 0.

#define MOTOR_PIN 6   // P0.06
#define SDA_PIN   17  // P0.17
#define SCL_PIN   20  // P0.20
#define BUTTON_PIN 29 // P0.29 (A2/D2 on many Nice!Nano/SuperMini pinouts - check yours!)

// Optional: Piezo Buzzer for Startup Melody
// Uncomment to enable
#define BUZZER_PIN 24 // P0.24 (Check available pins)

// ===== Haptic Feedback Thresholds (in mm) =====
#define DIST_FAR    2000      // > 2m: No vibration
#define DIST_MEDIUM 1000      // 1m - 2m: Slow pulse
#define DIST_CLOSE  300       // 30cm - 1m: Fast pulse
                              // < 30cm: Continuous vibration

// ===== Sensor Settings =====
#define I2C_FREQUENCY 400000   // 400 kHz
#define SENSOR_RESOLUTION 4*4  // 4x4 zones (16 zones)
#define SENSOR_RANGING_FREQ 15 // 15 Hz update rate

// ===== Modes =====
enum OperationMode {
  MODE_NAVIGATION, // Wide angle, safety bubble, pulsed feedback
  MODE_PRECISION   // Narrow angle, wall following, analog feedback
};

// ===== Power Management =====
#define AUTO_OFF_MS 300000  // 5 Minutes (5 * 60 * 1000)
                            // Device turns off if no vibration & no button press for this time.

// ===== Find Me Features =====
// Uncomment to enable searching for a Bluetooth Selfie Button (e.g. "AB Shutter3")
#define ENABLE_SELFIE_FINDER 
#define SELFIE_BUTTON_NAME "AB Shutter3" // Common name for cheap remotes
#define SCAN_INTERVAL_MS 4000  // Scan every 4 seconds
#define SCAN_WINDOW_MS   200   // Scan for 200ms (Duty Cycle ~5%)
