# Haptic-Horizon
The shin-saving, forehead-protecting sixth sense designed exclusively for Samira. Go batgirl mode! 🦇

## Overview
Haptic Horizon is a wearable navigational aid that uses Time-of-Flight (ToF) laser technology to translate the distance of obstacles into haptic feedback. The closer the object, the faster the vibration.

## Hardware
- **Microcontroller**: SuperMini NRF52840 (Nice!Nano compatible)
- **Distance Sensor**: VL53L5CX (Time-of-Flight 8x8 Multizone)
- **Output**: Vibration Motor Module (Breakout Board)
- **Audio**: Passive Piezo Buzzer
- **Input**: Momentary Push Button
- **Power**: 3.7V LiPo Battery (SuperMini has built-in charging via B+/B- pads)

## Power Supply (Important!)
*   **Recommended:** 3.7V LiPo/Li-Ion Battery.
    *   **Type:** 18650 Cell (2500mAh+) is excellent for a comfortable, grip-friendly handle design.
    *   Connect to **B+** and **B-**.
    *   Charges automatically via USB-C.
*   **Alternative:** Flat LiPo Pouch Cell (for slimmer designs).
*   **NOT Recommended:** Coin Cells (CR2032/CR2450). They cannot handle the current spikes (~100mA).

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

### 3. "Find Me" Feature (Bluetooth App)
If the device is lost (even in Auto-Off mode), it can be found using a smartphone.
1.  Open a BLE App (e.g., **nRF Connect** or **Adafruit Bluefruit**).
2.  Connect to **"Haptic Horizon"**.
3.  Select the **UART Service**.
4.  Send the character **'B'** (or 'F').
5.  🎵 The device will play a loud **"Here I Am"** melody.

### 4. "Selfie Button" Finder (Tactile Remote)
For a phone-free experience, you can use a cheap Bluetooth Camera Shutter remote (e.g., "AB Shutter3").
1.  Enable `#define ENABLE_SELFIE_FINDER` in `include/config.h`.
2.  Set the name of your remote in `SELFIE_BUTTON_NAME` (check via phone first).
3.  When the device is in **Auto-Off** mode, it scans for the remote every 4 seconds.
4.  Turn on or press the remote button.
5.  🎵 The device plays *La Marseillaise* for ~20 seconds.

## Configuration (`include/config.h`)
You can customize the device behavior by editing `include/config.h`:
*   **Haptic Thresholds:** Adjust distances for vibration intensity.
*   **Auto-Off Timer:** Default is 5 minutes (`300000` ms).
*   **Selfie Finder:** Uncomment `#define ENABLE_SELFIE_FINDER` to enable the remote scanner.
*   **Scanner Settings:** Adjust `SCAN_INTERVAL_MS` to trade off reaction time vs. battery life.

## Wiring (Vibration Motor Module)
Connect the Vibration Motor Breakout Board as follows:

*   **VCC**: Connect to **3.3V** or **BAT+**
*   **GND**: Connect to **GND**
*   **IN / SIG**: Connect to **P0.06**

*Note: The breakout board already contains the necessary transistor driver and protection diode.*

## Battery Life Estimation
*Estimates based on a **2500 mAh 18650 Cell**.*

| Scenario | Avg. Current | Estimated Runtime |
| :--- | :--- | :--- |
| **Active Use** | ~40-80 mA | **~40-50 Hours** |
| **BLE Standby** (App Find only) | ~0.5 mA | **~6-8 Months** |
| **BLE Standby + Selfie Scanner** | ~1.0 mA | **~3-4 Months** |
| **Deep Sleep** (Off) | ~0.05 mA | **Years** |

## Status
✅ **Feature Complete** - Ready for Assembly.


