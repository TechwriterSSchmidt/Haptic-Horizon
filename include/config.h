#pragma once
#include <Arduino.h>

// ===== Pin Definitions (SuperMini NRF52840 / Nice!Nano) =====
// We use the direct GPIO numbers (P0.xx) because the board variant 
// might map "D1" to something else.
// For safety, we define them as integers matching the GPIO port 0.

#define SDA_PIN   17  // P0.17 (Primary I2C: ToF, IMU, Haptic)
#define SCL_PIN   20  // P0.20 (Primary I2C)

// Secondary I2C Bus for Thermal Camera (High Traffic)
#define SDA1_PIN  6   // P0.06 (Dedicated for MLX90640)
#define SCL1_PIN  8   // P0.08 (Dedicated for MLX90640)

#define BUTTON_PIN 29 // P0.29 (A2/D2 on many Nice!Nano/SuperMini pinouts - check yours!)
#define TRIGGER_PIN 31 // P0.31 (Second button for Heat Vision "Trigger")

// Battery Measurement Pin (Requires Voltage Divider if Bat > 3.3V!)
// Connect Battery (+) -> 100k Resistor -> A0 -> 100k Resistor -> GND
#define BATTERY_PIN 2 // P0.02 (A0)

// ===== Haptic Driver =====
#define ENABLE_DRV2605 // Enable DRV2605L Driver

// ===== Haptic Feedback Thresholds (in mm) =====
#define DIST_FAR    2500     // > 2.5m No vibration
#define DIST_MEDIUM 1000      // 1m - 2m: Slow pulse
#define DIST_CLOSE  300       // 30cm - 1m: Fast pulse
                              // < 30cm: Continuous vibration

#define DIST_DROPOFF_THRESHOLD 2000 // In Drop-off mode: If distance > 2m, ALARM!

// ===== Sensor Settings =====
#define I2C_FREQUENCY 400000   // 400 kHz
#define SENSOR_RESOLUTION 4*4  // 4x4 zones (16 zones)
#define SENSOR_RANGING_FREQ 15 // 15 Hz update rate

// ===== Smart Terrain Settings (Virtual Cane) =====
#define HAND_HEIGHT_MM 1100    // Assumed height of the device above ground (in mm)
#define TOLERANCE_MM 150       // Tolerance for ground unevenness (+/- 15cm is considered flat)

// Haptic Patterns
#define HAPTIC_NONE 0
#define HAPTIC_WALL 1      // Continuous vibration (Obstacle ahead)
#define HAPTIC_DROPOFF 2   // Fast Strobe (Drop-off / Hole detected)
#define HAPTIC_OBSTACLE 3  // Slow Pulse (Step up / Curb detected)
#define HAPTIC_STAIRS_UP 4 // Ascending Pulse (Stairs Up detected)
#define HAPTIC_GAP 5       // Double Click (Door/Gap found)
#define HAPTIC_GLASS 6     // Sharp Tick (Glass/Mirror warning)

// Gap Hunter Settings
#define GAP_WIDTH_MIN_MM 600   // Minimum width for a door (standard is ~800mm)
#define GAP_DEPTH_DIFF_MM 1000 // The gap must be at least 1m deeper than the frame

// Stair Detection Thresholds
#define STAIR_STEP_MIN_HEIGHT 100 // Minimum height difference for a step (mm)
#define STAIR_STEP_MAX_HEIGHT 300 // Maximum height difference for a step (mm)

// ===== Modes =====
enum OperationMode {
  MODE_SMART_TERRAIN, // Uses IMU + ToF to detect walls, drop-offs, and obstacles automatically
  MODE_PRECISION,     // Manual override: Narrow angle, wall following
  MODE_HEAT_VISION    // Uses MLX90640 Thermal Camera to find heat sources
};

// ===== Drop Beacon (Fall Detection) =====
#define DROP_IMPACT_THRESHOLD 2.5 // G-force threshold to detect a fall/impact (approx 2.5G)
#define DROP_WAIT_TIME_MS 5000    // Wait 5 seconds after fall before alarming (gives time to pick it up)
#define DROP_ALARM_DURATION_MS 30000 // How long the alarm sounds (30 seconds)

// ===== MLX90640 Thermal Camera Settings =====
// Default I2C address is 0x33
#define HEAT_THRESHOLD_C 28   // Temperature in Celsius to trigger haptic feedback (Body heat is ~37C, surface temp lower)
#define HEAT_MAX_C 40         // Temperature for maximum vibration intensity

// ===== IMU Settings (BMI160) =====
#define BMI160_I2C_ADDR 0x69 // Default is often 0x68 or 0x69. Check your module!
#define MOUNTING_PITCH_OFFSET 0 // Calibration: Add +/- degrees if the sensor is not mounted perfectly flat
#define TILT_THRESHOLD_DOWN -45 // Degrees. (Used for internal logic if needed, but main logic is now continuous)
#define MOTION_THRESHOLD_GYRO 5 // Degrees/sec. If gyro < this for AUTO_OFF_MS, sleep.

// ===== DRV2605L Haptic Driver Settings =====
// Uncomment to enable the DRV2605L I2C Driver
#define ENABLE_DRV2605

// Uncomment to enable STEREO HAPTICS (Requires 2nd DRV2605L on Secondary I2C Bus)
// This allows for Left/Right directional feedback.
// Connect the second driver to SDA1/SCL1.
#define ENABLE_STEREO_HAPTICS 

// Uncomment if you are using a Linear Resonant Actuator (LRA) motor.
// Default is ERM (Eccentric Rotating Mass - the standard offset weight motor).
#define DRV2605_MOTOR_TYPE_LRA 

// Motor Voltage Setting (Important for 1.2V motors!)
// The DRV2605L can limit the output voltage to protect the motor.
// Set this to your motor's rated voltage (e.g., 1.2, 2.0, 3.0).
#define MOTOR_RATED_VOLTAGE 1.2 
#define MOTOR_CLAMP_VOLTAGE 1.2 // Usually same as rated, or slightly higher for overdrive

// ===== Power Management =====
#define AUTO_OFF_MS 120000  // 2 Minutes (2 * 60 * 1000)
                            // Device turns off if no movement (IMU) & no button press for this time.
#define MOVEMENT_THRESHOLD_MM 100 // Minimum distance change to count as "Activity" (prevents staying on when on table)

#define WATCHDOG_TIMEOUT_SEC 3 // 3 Seconds Timeout for system freeze protection

// ===== Audio Volume (1-10) =====
#define VOL_STARTUP     8
#define VOL_SHUTDOWN    5
#define VOL_MODE_CHANGE 7
#define VOL_ALARM       10
#define VOL_BATTERY     8  // Volume for battery announcement

// ===== Find Me Features =====
// Uncomment to enable searching for a Bluetooth Selfie Button (e.g. "AB Shutter3")
#define ENABLE_SELFIE_FINDER 
#define SELFIE_BUTTON_NAME "AB Shutter3" // Common name for cheap remotes
#define SCAN_INTERVAL_MS 4000  // Scan every 4 seconds
#define SCAN_WINDOW_MS   200   // Scan for 200ms (Duty Cycle ~5%)
