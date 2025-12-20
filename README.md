# Haptic-Horizon
The shin-saving, forehead-protecting sixth sense designed exclusively for Samira. Go batgirl mode!

## Overview
Haptic Horizon is a wearable navigational aid that uses Time-of-Flight (ToF) laser technology to translate the distance of obstacles into haptic feedback. The closer the object, the faster the vibration.

## Hardware
- **Microcontroller**: SuperMini NRF52840 (Nice!Nano compatible)
- **Distance Sensor**: VL53L5CX (Time-of-Flight 8x8 Multizone)
- **Output**: Vibration Motor (requires Transistor Driver!)
- **Power**: 3.7V LiPo Battery (SuperMini has built-in charging via B+/B- pads)

## Circuit Diagram (Motor Driver)
**WARNING:** Do NOT connect a raw motor directly to the GPIO pin!

### Option A: Using a Breakout Board (Recommended)
The easiest way is to use a **Vibration Motor Module** (e.g., from DFRobot, Adafruit, or generic). These boards already have the transistor and diode built-in.
*   **VCC**: Connect to 3.3V or BAT+
*   **GND**: Connect to GND
*   **IN / SIG**: Connect to **P0.06** (PWM works perfectly!)

### Option B: Custom Circuit (DIY)
If you have a raw motor, use a simple NPN Transistor (e.g., BC547, 2N2222) or MOSFET.

```text
                 + 3.3V (or BAT+)
                   |
                   +-------+
                   |       |
                 ( M )   (Diode 1N4148, Cathode to +)
                   |       |
                   +-------+
                   |
               C / D (Collector/Drain)
GPIO P0.06 ----[ 1k Resistor ]---- B / G (Base/Gate)
(Motor Pin)        |
               E / S (Emitter/Source)
                   |
                  GND
```

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
| Mode Button | P0.29 (A2) | Switch Mode (GND) |
| Buzzer (Optional) | P0.24 | Piezo (+) to Pin, (-) to GND |

## Battery Life Estimation (2500 mAh LiPo)
*Estimates based on continuous sensor operation at 15Hz.*

| Scenario | Avg. Current | Estimated Runtime |
| :--- | :--- | :--- |
| **Open Space** (No Vibration) | ~55 mA | **~45 Hours** |
| **Mixed Use** (Normal Walking) | ~70 mA | **~35 Hours** |
| **Heavy Use** (Constant Feedback) | ~120 mA | **~20 Hours** |
| **Deep Sleep** (Auto-Off) | ~0.05 mA | **Years** |

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
