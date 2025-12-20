# Haptic-Horizon
The shin-saving, forehead-protecting sixth sense designed exclusively for Samira. Go batgirl mode!

## Overview
Haptic Horizon is a wearable navigational aid that uses Time-of-Flight (ToF) laser technology to translate the distance of obstacles into haptic feedback. The closer the object, the faster the vibration.

## Hardware
- **Microcontroller**: SuperMini NRF52840 (Nice!Nano compatible)
- **Distance Sensor**: VL53L5CX (Time-of-Flight 8x8 Multizone)
- **Output**: Vibration Motor (connected via PWM or simple GPIO)
- **Power**: 3.7V LiPo Battery (SuperMini has built-in charging via B+/B- pads)

## Functionality
1.  **Distance Measurement**: The VL53L5CX measures the distance in a 4x4 grid (Wide Field of View).
2.  **Safety Logic**: The system calculates the distance to the *closest* object in any of the 16 zones. This creates a "protective bubble" effect, detecting obstacles even if they are not perfectly centered.
3.  **Haptic Feedback**:
    - **Far (> 2m)**: No vibration.
    - **Medium (1m - 2m)**: Gentle/Slow pulses.
    - **Close (< 1m)**: Strong/Fast pulses.
    - **Critical (< 30cm)**: Continuous vibration.

## Pinout (SuperMini NRF52840)
| Component | SuperMini Pin | Description |
|-----------|---------------|-------------|
| VL53L5CX SDA | P0.17 (D2) | I2C SDA |
| VL53L5CX SCL | P0.20 (D3) | I2C SCL |
| Vibration Motor | P0.06 (D1) | PWM capable |
| Battery + | B+ | LiPo Positive |
| Battery - | B- | LiPo Negative |

## Setup
1.  Open in VS Code with PlatformIO.
2.  Build and Upload.

## Hardware
- **Controller:** Seeed Xiao ESP32C3 (or similar small form factor)
- **Sensor:** VL53L1X (Long Range ToF Sensor, up to 4m)
- **Feedback:** Coin Vibration Motor (PWM controlled)
- **Power:** LiPo Battery

## How it works
1.  **Scan:** The VL53L1X continuously measures the distance to objects in front of the user.
2.  **Process:** The ESP32 maps the distance (e.g., 0.1m to 4.0m) to a vibration intensity/pattern.
3.  **Buzz:** 
    - **Far away (> 2m):** No vibration (Safe zone).
    - **Approaching (1m - 2m):** Gentle, slow pulses.
    - **Close (< 1m):** Rapid, strong buzzing.
    - **IMMINENT BONK (< 30cm):** Continuous alarm vibration.

## Status
🚧 **Under Construction** - Initial scaffolding phase.
