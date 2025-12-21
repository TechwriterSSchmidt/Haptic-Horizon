# Haptic-Horizon
The shin-saving, forehead-protecting sixth sense designed exclusively for Samira. Go batgirl mode! 🦇

## Overview
Haptic Horizon is a smart wearable navigational aid that fuses **LiDAR (ToF)**, **Thermal Imaging**, and **Motion Sensing** to translate the environment into a rich haptic language.

Unlike simple proximity sensors, Haptic Horizon understands context:
*   **Smart Terrain:** Detects walls, drop-offs, and stairs using 3D gradient analysis.
*   **Heat Vision:** Identifies people, pets, and electronics by combining heat signatures with object size.
*   **Safety First:** Includes a "Drop Beacon" that calls for help if the device falls.

It's designed to be a discreet, powerful companion that translates distance and temperature into intuitive vibration patterns. The closer or hotter the object, the more distinct the feedback.

## Support my projects

Your tip motivates me to continue developing nerdy stuff for the DIY community. Thank you very much for your support!

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/D1D01HVT9A)

## Hardware
- **Microcontroller**: SuperMini NRF52840 (Nice!Nano compatible)
- **Distance Sensor**: VL53L5CX (Time-of-Flight 8x8 Multizone)
- **Thermal Camera**: MLX90640-BAB (32x24 Infrared Array)
- **IMU**: BMI160 (Gyroscope + Accelerometer)
- **Haptic Driver**: DRV2605L (Optional - for advanced waveforms & LRA support)
- **Output**: Vibration Motor (LRA Coin Type recommended, 1.2V, max 100mA)
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
| VL53L5CX SDA | P0.17 (D2) | I2C SDA (Shared) |
| VL53L5CX SCL | P0.20 (D3) | I2C SCL (Shared) |
| AMG8833 SDA | P0.17 (D2) | I2C SDA (Shared) |
| AMG8833 SCL | P0.20 (D3) | I2C SCL (Shared) |
| BMI160 SDA | P0.17 (D2) | I2C SDA (Shared) |
| BMI160 SCL | P0.20 (D3) | I2C SCL (Shared) |
| DRV2605L SDA | P0.17 (D2) | I2C SDA (Shared) |
| DRV2605L SCL | P0.20 (D3) | I2C SCL (Shared) |
| Vibration Motor | P0.06 (D1) | PWM (Direct) OR DRV2605L Output |
| Buzzer | P0.24 | Piezo (+) to Pin, (-) to GND |
| Mode Button | P0.29 (A2) | Button to GND (Internal Pullup) |
| Trigger Button | P0.31 (D6) | Button to GND (Internal Pullup) |
| Battery + | B+ | LiPo Positive |
| Battery - | B- | LiPo Negative |

## User Guide

### 1. Power On / Off
*   **Start / Wake Up:** Press any button **twice quickly** (Double Tap).
    *   *Signal:* "La Marseillaise" melody.
*   **Auto-Off:** Automatically turns off after **2 minutes** of inactivity.
    *   **Smart Detection:** Stays on as long as you move (Gyroscope detection). Turns off if placed on a table or held still.
    *   Sound: *Windows Shutdown* (Classic)
    *   The device enters **Deep Sleep** (System OFF) to save power and prevent accidental wakeups.
    *   *Note:* Motion alone will NOT wake the device. You must double-tap a button.

### 2. Modes (Toggle via Button)
Press the **Mode Button** to switch between *Smart Terrain* and *Precision Mode*.

#### A. Smart Terrain Mode (Default)
Uses **Gradient Analysis** (Computer Vision) to understand the environment in 3D. The device compares the upper and lower zones of the sensor to distinguish between walls, stairs, and drop-offs.

**The Haptic Language:**

| Situation | Sensor Pattern | Haptic Feedback (LRA) | Feeling |
| :--- | :--- | :--- | :--- |
| **Wall / Obstacle** | Object ahead (ignoring ground) | **Continuous Buzz** | Like a force field. Stronger = Closer. |
| **Drop-off / Hole** | Ground suddenly disappears | **Ramp Down** | A "falling" sensation. Warning! |
| **Stairs Up** | Distance increases in steps | **Ramp Up** | A "rising" sensation. |
| **Curb / Trip Hazard** | Small obstacle on ground | **Soft Bump** | A gentle nudge. |
| **Gap / Door** | Open space between obstacles | **Double Click** | A distinct "click-click". |
| **Bat-in-danger Mode** | Confusing reflections (Glass/Mirror) | **Sharp Tick** | A short, sharp warning tick. |
| **Flat Ground** | Consistent gradient | **Silence** | Safe to walk. |

#### B. Precision Mode
*   **Sound:** *Zoom In* (Ascending Tones)
*   **Function:** Scans only the center point (Tunnel Vision).
*   **Feedback:** **Sharp Clicks** (Geiger-Counter Style). Faster clicking = Closer.
*   **Use Case:** Finding door handles, locating narrow gaps, or checking specific objects.

#### C. Heat Vision Mode (Trigger Activated)
*   **Activation:** Press and hold the **Trigger Button** (Abzugsfinger). Release to return to previous mode.
*   **Sound:** *High Pitch Pulse* (On Activate) / *Low Pitch* (On Release)
*   **Function:** Uses Sensor Fusion (Thermal + ToF) to identify heat sources.
*   **Feedback:**
    *   **Human (Narrow & Hot):** Slow **Heartbeat** (*Bumm... Bumm...*).
    *   **Monitor / Machine (Wide & Hot):** Mechanical **Double-Tick** (*Tick-Tick...*).
    *   **Small Object (Cup/Cat):** Fast **Geiger Counter** (*Tickticktick*).
*   **Use Case:** Finding people in the dark, checking if electronics are running, locating pets.

### 3. Battery Check
*   **Long Press (> 2s):** The device announces the battery level.
    *   4 Beeps: Full (> 4.0V)
    *   3 Beeps: Good (> 3.7V)
    *   2 Beeps: Low (> 3.4V)
    *   1 Long Beep: Critical (< 3.4V)

### 4. Calibration (IMU)
If the device is not detecting the ground correctly (e.g., false alarms on flat ground), you can recalibrate the "Zero" position.
1.  Place the device **flat on a table** (or the surface you want to define as "level").
2.  Press and **hold the button for 10 seconds**.
3.  Wait for the **Success Melody** (Major Arpeggio).
4.  The new calibration is saved permanently.

### 5. "Find Me" Feature (Bluetooth App)
If the device is lost (even in Auto-Off mode), it can be found using a smartphone.
1.  Open a BLE App (e.g., **nRF Connect** or **Adafruit Bluefruit**).
2.  Connect to **"Haptic Horizon"**.
3.  Select the **UART Service**.
4.  Send the character **'B'** (or 'F').
5.  The device will play a loud **"Here I Am"** melody.

### 6. Safety Features
*   **Drop Beacon:** If the device detects a hard fall (Impact > 2.5G), it waits 5 seconds. If not picked up, it triggers a loud alarm and flashing haptics for 30 seconds to help you find it.


### 5. "Selfie Button" Finder (Tactile Remote)
For a phone-free experience, you can use a cheap Bluetooth Camera Shutter remote (e.g., "AB Shutter3").
1.  Enable `#define ENABLE_SELFIE_FINDER` in `include/config.h`.
2.  Set the name of your remote in `SELFIE_BUTTON_NAME` (check via phone first).
3.  When the device is in **Auto-Off** mode, it scans for the remote every 4 seconds.
4.  Turn on or press the remote button.
5.  The device plays *La Marseillaise* for ~20 seconds.

## Documentation
For a printable, easy-to-read guide for the user, see [Docs/QUICK_REFERENCE.md](Docs/QUICK_REFERENCE.md).

## Configuration (`include/config.h`)
You can customize the device behavior by editing `include/config.h`:
*   **Haptic Driver:** Uncomment `#define ENABLE_DRV2605` to use the DRV2605L chip instead of direct PWM.
*   **Motor Type:** If using DRV2605L, uncomment `#define DRV2605_MOTOR_TYPE_LRA` for Linear Resonant Actuators. Default is ERM (Eccentric Rotating Mass).
*   **Haptic Thresholds:** Adjust distances for vibration intensity.
*   **Auto-Off Timer:** Default is 5 minutes (`300000` ms).
*   **Selfie Finder:** Uncomment `#define ENABLE_SELFIE_FINDER` to enable the remote scanner.
*   **Scanner Settings:** Adjust `SCAN_INTERVAL_MS` to trade off reaction time vs. battery life.

## Wiring (Vibration Motor)

### Option A: Direct PWM (Basic)
Connect a Vibration Motor Breakout Board (with transistor):
*   **VCC**: Connect to **3.3V** or **BAT+**
*   **GND**: Connect to **GND**
*   **IN / SIG**: Connect to **P0.06**

### Option B: DRV2605L Driver (Advanced)
Connect the DRV2605L Breakout Board:
*   **VIN**: Connect to **3.3V**
*   **GND**: Connect to **GND**
*   **SDA**: Connect to **P0.17**
*   **SCL**: Connect to **P0.20**
*   **Motor**: Connect motor wires to the output pads on the DRV2605L.

## Battery Life Estimation
*Estimates based on a **2500 mAh 18650 Cell**.*

| Scenario | Avg. Current | Estimated Runtime |
| :--- | :--- | :--- |
| **Active Use (LRA Haptics)** | ~80-120 mA | **~20-30 Hours** |
| **BLE Standby** (App Find only) | ~0.5 mA | **~6-8 Months** |
| **BLE Standby + Selfie Scanner** | ~1.0 mA | **~3-4 Months** |
| **Deep Sleep** (Off) | ~0.05 mA | **Years** |

## Contributing

Feel free to open issues or submit pull requests for improvements!

## License

This project is open source. Feel free to use and modify as needed.

## Credits

Created to help Samira navigate the physical obstacles of her life more easily, with a casually elegant flick of her wrist. Batgirl-style!



