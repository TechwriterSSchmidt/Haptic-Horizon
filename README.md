# Haptic-Horizon
The shin-saving, forehead-protecting sixth sense designed exclusively for Samira. Go batgirl mode!

## Overview
Haptic Horizon is a wearable navigational aid that uses Time-of-Flight (ToF) laser technology to translate the distance of obstacles into haptic feedback. The closer the object, the faster the vibration.

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
