# Haptic-Horizon
The shin-saving, forehead-protecting sixth sense designed exclusively for Samira. Go batgirl mode! 🦇

## Overview
Haptic Horizon is a smart wearable **Electronic Travel Aid (ETA)** that fuses **LiDAR (ToF)**, **Thermal Imaging**, and **Motion Sensing** to translate the environment into a rich haptic language.

Unlike simple proximity sensors, Haptic Horizon understands context:
*   **Smart Terrain:** Detects walls, drop-offs, and stairs using 3D gradient analysis.
*   **Heat Vision:** Identifies people, pets, and electronics by combining heat signatures with object size.
*   **Safety First:** Includes a "Drop Beacon" that calls for help if the device falls.

It's designed to be a discreet, powerful companion that translates distance and temperature into intuitive vibration patterns. The closer or hotter the object, the more distinct the feedback.

## Design Philosophy & Research
Our design is informed by research into assistive technologies (such as [PMC5375851](https://pmc.ncbi.nlm.nih.gov/articles/PMC5375851/)), which highlights the importance of:
1.  **Silent UI:** We strictly avoid audio feedback (beeps/voice) to prevent masking environmental sounds that are critical for situational awareness.
2.  **Cognitive Load Management:** Instead of complex "haptic Morse code," we use intuitive "Hapticons" (Haptic Icons) that mimic physical sensations (e.g., a "ramp up" feeling for startup, distinct pulses for battery levels).
3.  **Vision Substitution:** Haptic Horizon acts as a near-field environmental scanner (ETA), complementing the white cane rather than replacing it.

## Support my projects

Your tip motivates me to continue developing nerdy stuff for the DIY community. Thank you very much for your support!

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/D1D01HVT9A)

## Hardware
- **Microcontroller**: SuperMini NRF52840 (Nice!Nano compatible)
- **Distance Sensor**: VL53L5CX (Time-of-Flight 8x8 Multizone)
- **Thermal Camera**: MLX90640-BAB (32x24 Infrared Array)
- **IMU**: BMI160 (Gyroscope + Accelerometer)
- **Haptic Driver**: DRV2605L (Required - for advanced waveforms & LRA support)
- **Output**: LRA Vibration Motor (Connected to DRV2605L)
- **Input**: Mode Button, Trigger Button
- **Power**: 3.7V LiPo Battery (SuperMini has built-in charging via B+/B- pads)

## Power Supply (Important!)
*   **Recommended:** 3.7V LiPo/Li-Ion Battery.
    *   **Type:** 18650 Cell (2500mAh+) is excellent for a comfortable, grip-friendly handle design.
    *   Connect to **B+** and **B-**.
    *   Charges automatically via USB-C.
*   **Alternative:** Flat LiPo Pouch Cell (for slimmer designs).
*   **NOT Recommended:** Coin Cells (CR2032/CR2450). They cannot handle the current spikes (~100mA).

## Pinout (SuperMini NRF52840)
| GPIO Pin | Function | Connected Component(s) |
| :--- | :--- | :--- |
| **P0.17** | I2C SDA (Primary) | VL53L5CX, BMI160, DRV2605L |
| **P0.20** | I2C SCL (Primary) | VL53L5CX, BMI160, DRV2605L |
| **P0.06** | I2C SDA (Secondary) | MLX90640 (Thermal Camera) |
| **P0.08** | I2C SCL (Secondary) | MLX90640 (Thermal Camera) |
| **P0.29** | Digital Input | Mode Button |
| **P0.31** | Digital Input | Trigger Button |
| **P0.02** | Analog Input | Battery Voltage Divider |
| **B+** | Power | LiPo Battery Positive |
| **B-** | Power | LiPo Battery Negative |

## User Guide

### 1. Power On / Off
*   **Start / Wake Up:** Press any button **twice quickly** (Double Tap).
    *   *Signal:* **Haptic Ramp Up** (Vibration increases).
*   **Auto-Off:** Automatically turns off after **2 minutes** of inactivity.
    *   **Smart Detection:** Stays on as long as you move (Gyroscope detection). Turns off if placed on a table or held still.
    *   *Signal:* **Haptic Ramp Down** (Vibration fades out).
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
*   **Function:** Scans only the center point (Tunnel Vision).
*   **Feedback:** **Sharp Clicks** (Geiger-Counter Style). Faster clicking = Closer.
*   **Use Case:** Finding door handles, locating narrow gaps, or checking specific objects.

#### C. Heat Vision Mode (Trigger Activated)
*   **Activation:** Press and hold the **Trigger Button** (Abzugsfinger). Release to return to previous mode.
*   **Function:** Uses Sensor Fusion (Thermal + ToF) to identify heat sources.
*   **Feedback:**
    *   **Human (Narrow & Hot):** Slow **Heartbeat** (*Bumm... Bumm...*).
    *   **Monitor / Machine (Wide & Hot):** Mechanical **Double-Tick** (*Tick-Tick...*).
    *   **Small Object (Cup/Cat):** Fast **Geiger Counter** (*Tickticktick*).
*   **Use Case:** Finding people in the dark, checking if electronics are running, locating pets.

### 3. Status Check (Battery & Distance)
*   **Long Press (> 2s) on Mode Button:** The device announces the battery status via haptic pulses.
    *   **4 Pulses:** Full
    *   **3 Pulses:** Good
    *   **2 Pulses:** Low
    *   **1 Long Pulse:** Critical

### 4. Calibration (IMU)
If the device is not detecting the ground correctly (e.g., false alarms on flat ground), you can recalibrate the "Zero" position.
1.  Place the device **flat on a table** (or the surface you want to define as "level").
2.  Press and **hold the Mode Button for 10 seconds**.
3.  Wait for the **Success Triple Click**.
4.  The new calibration is saved permanently.

### 6. "Find Me" Feature (Bluetooth App)
If the device is lost (even in Auto-Off mode), it can be found using a smartphone.
1.  Open a BLE App (e.g., **nRF Connect** or **Adafruit Bluefruit**).
2.  Connect to **"Haptic Horizon"**.
3.  Select the **UART Service**.
4.  Send the character **'B'** (or 'F').
5.  The device will vibrate strongly (Pulsing Alarm).

### 7. Safety Features
*   **Drop Beacon:** If the device detects a hard fall (Impact > 2.5G), it waits 5 seconds. If not picked up, it triggers a strong pulsing vibration alarm for 30 seconds to help you find it on the floor.

### 8. "Selfie Button" Finder (Tactile Remote)
For a phone-free experience, you can use a cheap Bluetooth Camera Shutter remote (e.g., "AB Shutter3").
1.  Enable `#define ENABLE_SELFIE_FINDER` in `include/config.h`.
2.  Set the name of your remote in `SELFIE_BUTTON_NAME` (check via phone first).
3.  When the device is in **Auto-Off** mode, it scans for the remote every 4 seconds.
4.  Turn on or press the remote button.
5.  The device will vibrate strongly (Pulsing Alarm).

## Documentation
For a printable, easy-to-read guide for the user, see [Docs/QUICK_REFERENCE.md](Docs/QUICK_REFERENCE.md).

## Configuration (`include/config.h`)
You can customize the device behavior by editing `include/config.h`:
*   **Haptic Driver:** `ENABLE_DRV2605` is enabled by default (Required).
*   **Motor Type:** Uncomment `#define DRV2605_MOTOR_TYPE_LRA` for Linear Resonant Actuators. Default is ERM (Eccentric Rotating Mass).
*   **Haptic Thresholds:** Adjust distances for vibration intensity.
*   **Auto-Off Timer:** Default is 2 minutes (`120000` ms).
*   **Selfie Finder:** Uncomment `#define ENABLE_SELFIE_FINDER` to enable the remote scanner.
*   **Scanner Settings:** Adjust `SCAN_INTERVAL_MS` to trade off reaction time vs. battery life.

## Wiring (Vibration Motor)

### DRV2605L Driver (Required)
Connect the DRV2605L Breakout Board:
*   **VIN**: Connect to **3.3V**
*   **GND**: Connect to **GND**
*   **SDA**: Connect to **P0.17** (Primary Bus)
*   **SCL**: Connect to **P0.20** (Primary Bus)
*   **Motor**: Connect LRA motor wires to the output pads on the DRV2605L.

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



