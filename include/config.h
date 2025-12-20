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

// ===== Haptic Feedback Thresholds (in mm) =====
#define DIST_FAR    2000      // > 2m: No vibration
#define DIST_MEDIUM 1000      // 1m - 2m: Slow pulse
#define DIST_CLOSE  300       // 30cm - 1m: Fast pulse
                              // < 30cm: Continuous vibration

// ===== Sensor Settings =====
#define I2C_FREQUENCY 400000   // 400 kHz
#define SENSOR_RESOLUTION 4*4  // 4x4 zones (16 zones)
#define SENSOR_RANGING_FREQ 15 // 15 Hz update rate
