# Haptic-Horizon
The shin-saving, forehead-protecting sixth sense designed exclusively for Samira. Go batgirl mode!

## Overview
Haptic Horizon is a wearable navigational aid that uses Time-of-Flight (ToF) laser technology to translate the distance of obstacles into haptic feedback. The closer the object, the faster the vibration.

## Hardware
- **Microcontroller**: SuperMini NRF52840 (Nice!Nano compatible)
- **Distance Sensor**: VL53L5CX (Time-of-Flight 8x8 Multizone)
- **Output**: Vibration Motor (requires Transistor Driver!)
- **Audio**: Passive Piezo Buzzer
- **Input**: Momentary Push Button
- **Power**: 3.7V LiPo Battery (SuperMini has built-in charging via B+/B- pads)

## Power Supply (Important!)
*   **Recommended:** 3.7V LiPo Battery (e.g., 400mAh - 2500mAh).
    *   Connect to **B+** and **B-**.
    *   Charges automatically via USB-C.
*   **Alternative:** 3x AA Batteries (4.5V) connected to VCC/GND.
*   **NOT Recommended:** Coin Cells (CR2032/CR2450). They cannot handle the current spikes (~100mA) of the sensor and motor.

## Pinout (SuperMini NRF52840)
| Component | SuperMini Pin | Description |
|-----------|---------------|-------------|
| VL53L5CX SDA | P0.17 (D2) | I2C SDA |
| VL53L5CX SCL | P0.20 (D3) | I2C SCL |
| Vibration Motor | P0.06 (D1) | PWM capable (Needs Transistor!) |
| Buzzer | P0.24 | Piezo (+) to Pin, (-) to GND |
| Mode Button | P0.29 (A2) | Button to GND (Internal Pullup) |
| Battery + | B+ | LiPo Positive |
| Battery - | B- | LiPo Negative |

## User Guide

### 1. Power On / Off
*   **Start:** Connect power or reset.
    *   🎵 Sound: *La Marseillaise* (Opening)
*   **Auto-Off:** After **5 minutes** of inactivity (no obstacles < 2m, no button press).
    *   🎵 Sound: *Windows Shutdown* (Classic)
    *   The device enters **BLE Standby Mode** (Sensor OFF, Bluetooth ON).

### 2. Modes (Toggle via Button)
Press the button to switch between modes.
*   **Navigation Mode (Default):**
    *   🎵 Sound: *Zoom Out* (Descending Tones)
    *   Scans a wide area (Safety Bubble).
    *   Vibrates for the *closest* object in any direction.
*   **Precision Mode:**
    *   🎵 Sound: *Zoom In* (Ascending Tones)
    *   Scans only the center (Tunnel Vision).
    *   Used to find door handles or narrow gaps. Geiger-counter style clicking.

### 3. "Find Me" Feature (Bluetooth)
If the device is lost (even in Auto-Off mode), it can be found using a smartphone.
1.  Open a BLE App (e.g., **nRF Connect** or **Adafruit Bluefruit**).
2.  Connect to **"Haptic Horizon"**.
3.  Select the **UART Service**.
4.  Send the character **'B'** (or 'F').
5.  🎵 The device will play a loud **"Here I Am"** melody.

## Circuit Diagram (Motor Driver)
**WARNING:** Do NOT connect a raw motor directly to the GPIO pin! Use a Transistor.

```text
                 + 3.3V (or BAT+)
                   |
                   +-------+
                   |       |
                 ( M )   (Diode 1N4148)
                   |       |
                   +-------+
                   |
               C / D (Collector/Drain)
GPIO P0.06 ----[ 1k Resistor ]---- B / G (Base/Gate)
                   |
               E / S (Emitter/Source)
                   |
                  GND
```

## Battery Life Estimation
*Estimates based on 1000 mAh LiPo.*

| Scenario | Avg. Current | Estimated Runtime |
| :--- | :--- | :--- |
| **Active Use** | ~40-80 mA | **~15-20 Hours** |
| **BLE Standby** (Find Me) | ~0.5 mA | **~2-3 Months** |
| **Deep Sleep** (Off) | ~0.05 mA | **Years** |

## Status
✅ **Feature Complete** - Ready for Assembly.


