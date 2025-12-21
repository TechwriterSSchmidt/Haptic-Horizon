# Haptic Horizon
The shin-saving, forehead-protecting sixth sense designed exclusively for Samira. Go batgirl mode! 🦇

## Overview
Haptic Horizon is a smart wearable **Electronic Travel Aid (ETA)** that fuses **LiDAR (ToF)**, **Thermal Imaging**, and **Motion Sensing** to translate the environment into a rich haptic language.

Unlike simple proximity sensors, Haptic Horizon understands context:
*   **Smart Terrain:** Detects walls, drop-offs, and stairs using 3D gradient analysis.
*   **Heat Vision:** Identifies people, pets, and electronics by combining heat signatures with object size.
*   **Find Me:** Automatically vibrates if dropped so it can be easily located. Can also be triggered via Bluetooth (App or Camera Shutter).

It's designed to be a discreet, powerful companion that translates distance and temperature into intuitive vibration patterns. The closer or hotter the object, the more distinct the feedback.

## Support my projects

Your tip motivates me to continue developing nerdy stuff for the DIY community. Thank you very much for your support!

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/D1D01HVT9A)

## Design Philosophy & Research
Our design is informed by research into assistive technologies, which highlights the importance of:

1.  **Silent UI:** We strictly avoid audio feedback (beeps/voice) to prevent masking environmental sounds that are critical for situational awareness. This approach is supported by studies like [PMC5375851](https://pmc.ncbi.nlm.nih.gov/articles/PMC5375851/) and [Sensors 2023, 23(16), 7198](https://www.mdpi.com/1424-8220/23/16/7198), which emphasize the safety benefits of haptic-only feedback.
2.  **Cognitive Load Management:** Instead of complex "haptic Morse code," we use intuitive "Hapticons" (Haptic Icons) that mimic physical sensations (e.g., a "ramp up" feeling for startup, distinct pulses for battery levels).
3.  **Vision Substitution:** Haptic Horizon acts as a near-field environmental scanner (ETA), complementing the white cane rather than replacing it.
4.  **Multimodal Sensing:** We combine LiDAR (active) with Thermal Imaging (passive). As noted in [Sensors 2023, 23(16), 7198](https://www.mdpi.com/1424-8220/23/16/7198), thermal sensors can provide unique environmental cues (like detecting corners or occupancy) that other sensors might miss, while being energy efficient.
5.  **Anti-Fatigue Feedback:** Based on findings from [DIVA Portal (Haptic Obstacle Detector)](https://www.diva-portal.org/smash/get/diva2:1666232/FULLTEXT01.pdf), continuous vibration can desensitize the skin ("Haptic Fatigue"). We implement **"Pulsed Priority"** feedback (variable rhythm) instead of just variable intensity. This keeps the user alert without numbing their senses.
6.  **Semantic Navigation:** Inspired by [UZH Research](https://rpg.ifi.uzh.ch/docs/RAL18_Cioffi.pdf), we aim for "Semantic Haptics"—telling you *what* is there (Human vs. Wall), not just *that* something is there. This is realized through our "Heat Vision" mode.

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
    *   Charges automatically via USB-C.
*   **Alternative:** Flat LiPo Pouch Cell (for slimmer designs).

## Pinout (SuperMini NRF52840)
| GPIO Pin | Function | Connected Component(s) |
| :--- | :--- | :--- |
| **P0.17** | I2C SDA (Primary) | VL53L5CX, BMI160, DRV2605L |
| **P0.20** | I2C SCL (Primary) | VL53L5CX, BMI160, DRV2605L |
| **P0.06** | I2C SDA (Secondary) | MLX90640 (Thermal Camera), Optional 2nd DRV2605L |
| **P0.08** | I2C SCL (Secondary) | MLX90640 (Thermal Camera), Optional 2nd DRV2605L |
| **P0.29** | Digital Input | Mode Button |
| **P0.31** | Digital Input | Trigger Button |
| **P0.02** | Analog Input | Battery Voltage Divider |
| **B+** | Power | LiPo Battery Positive |
| **B-** | Power | LiPo Battery Negative |

## Advanced Features

### Stereo Haptics (Optional Upgrade)
For the ultimate intuitive experience, Haptic Horizon supports **Dual-Channel Feedback**.
*   **Hardware:** Connect a second DRV2605L to the secondary I2C bus (SDA1/SCL1).
*   **Positioning:** Place the two LRAs on opposite sides of the handle (180° offset).
    *   **LRA 1 (Left Channel):** Under the **Thumb** (for a flashlight grip).
    *   **LRA 2 (Right Channel):** Under the **Fingers**.
*   **Function:**
    *   **Navigation:** If a wall is closer on the left, the Thumb vibrates. If on the right, the Fingers vibrate.
    *   **Heat Vision:** If a person is to the left, the Thumb pulses.
    *   **Result:** Samira can navigate blindly by simply "steering away" from the vibration, without needing to scan back and forth.

## User Guide

### 1. Power On / Off
*   **Start / Wake Up:** Press any button **twice quickly** (Double Tap).
    *   *Signal:* **Haptic Ramp Up** (Vibration increases).
*   **Auto-Off:** Automatically turns off after **2 minutes** of inactivity.
    *   **Smart Detection:** Stays on as long as you move (Gyroscope detection). Turns off if placed on a table or held still.
    *   *Signal:* **Haptic Ramp Down** (Vibration fades out).
    *   The device enters **Deep Sleep** (System OFF) to save power and prevent accidental wakeups.
    *   *Note:* Motion alone will NOT wake the device. You must double-tap a button.

### 2. The Haptic Dictionary (Complete List)
This table lists every signal the device can produce.

| Category | Signal Name | Pattern (Rhythm) | Feeling | Meaning |
| :--- | :--- | :--- | :--- | :--- |
| **System** | Startup | **Ramp Up** | Rising Pitch | Device is waking up. |
| **System** | Shutdown | **Ramp Down** | Falling Pitch | Device is going to sleep (Auto-Off). |
| **System** | Battery Full | **4 Pulses** | *Tick-Tick-Tick-Tick* | Battery > 80%. |
| **System** | Battery Low | **2 Pulses** | *Tick-Tick* | Battery < 20%. Charge soon. |
| **System** | Calibration | **Triple Click** | *Click-Click-Click* | IMU Calibration saved. |
| **Terrain** | Wall / Obstacle | **Pulsed Buzz** | *Bzzz... Bzzz...* | Obstacle ahead. Faster = Closer. |
| **Terrain** | Drop-off | **Ramp Down** | Falling Sensation | Hole or stairs down! Stop! |
| **Terrain** | Stairs Up | **Ramp Up** | Rising Sensation | Stairs going up ahead. |
| **Terrain** | Gap / Door | **Double Click** | *Click-Click* | Open space found (Doorway). |
| **Terrain** | Glass Warning | **Sharp Tick** | *Tick* | Confusing reflection (Mirror/Glass). |
| **Heat** | Human | **Heartbeat** | *Bumm-Bumm...* | Person detected (Warm & Narrow). |
| **Heat** | Machine | **Double Tick** | *Tick-Tick...* | Electronics detected (Warm & Wide). |
| **Heat** | Small Object | **Geiger Counter** | *Trrrrr...* | Small heat source (Cup, Pet). |
| **Heat** | Hot Surface | **Fast Stutter** | *Bz-Bz-Bz-Bz* | **DANGER!** Very hot surface (>60°C). |
| **Alarm** | Drop Beacon | **Loud Strobe** | *BZZZ-BZZZ-BZZZ* | "I fell down! Pick me up!" |
| **Alarm** | Find Me | **Loud Strobe** | *BZZZ-BZZZ-BZZZ* | "Here I am!" (Triggered via App). |

### 3. Modes (Toggle via Button)
Press the **Mode Button** to switch between *Smart Terrain* and *Precision Mode*.

#### A. Smart Terrain Mode (Default)
Uses **Gradient Analysis** (Computer Vision) to understand the environment in 3D. The device compares the upper and lower zones of the sensor to distinguish between walls, stairs, and drop-offs.

#### B. Precision Mode
*   **Function:** Scans only the center point (Tunnel Vision).
*   **Feedback:** **Sharp Clicks** (Geiger-Counter Style). Faster clicking = Closer.
*   **Use Case:** Finding door handles, locating narrow gaps, or checking specific objects.

#### C. Heat Vision Mode (Trigger Activated)
*   **Activation:** Press and hold the **Trigger Button** (Abzugsfinger). Release to return to previous mode.
*   **Function:** Uses Sensor Fusion (Thermal + ToF) to identify heat sources.
*   **Feedback:** See "Heat" section in the Dictionary above.

### 4. Status Check (Battery & Distance)
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

### 6. Find Me & Drop Beacon Features
Haptic Horizon includes three ways to locate the device if it is dropped or misplaced. All methods trigger a **Loud Strobe Alarm** (Strong Pulsing Vibration).

#### A. Drop Beacon (Automatic)
*   **Trigger:** Detects a hard impact (> 2.5G) followed by stillness.
*   **Behavior:** Waits 5 seconds (grace period). If not picked up, it alarms for 30 seconds.
*   **Use Case:** You drop the device while walking and it rolls away.

#### B. Smartphone Finder (App)
*   **Trigger:** Bluetooth LE Command.
*   **How to use:**
    1.  Open **nRF Connect** or **Adafruit Bluefruit** app.
    2.  Connect to **"Haptic Horizon"**.
    3.  Send character **'B'** or **'F'** via UART.
*   **Use Case:** Device is lost in the sofa cushions.

#### C. "Selfie Button" Finder (Tactile Remote)
*   **Trigger:** A standard Bluetooth Camera Shutter (e.g., "AB Shutter3").
*   **How to use:** Press the button on the remote. The device wakes up and alarms.
*   **Setup:** Enable `#define ENABLE_SELFIE_FINDER` in `config.h` and set your remote's name.
*   **Use Case:** Finding the device without needing a smartphone app (Tactile & Fast).

## Documentation
For an easy-to-read guide for the user, see [Docs/QUICK_REFERENCE.md](Docs/QUICK_REFERENCE.md).

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
*Estimates based on a **2500 mAh 18650 Cell** with **Continuous Sensor Fusion** (LiDAR + Thermal always active).*

| Scenario | Avg. Current | Estimated Runtime |
| :--- | :--- | :--- |
| **Active (Single Motor)** | ~130 mA | **~18-20 Hours** |
| **Active (Stereo Motors)** | ~170 mA | **~14-16 Hours** |
| **BLE Standby** (App Find only) | ~0.5 mA | **~6-8 Months** |
| **BLE Standby + Selfie Scanner** | ~1.0 mA | **~3-4 Months** |
| **Deep Sleep** (Off) | ~0.05 mA | **Years** |

## Contributing

Feel free to open issues or submit pull requests for improvements!

## License

This project is open source. Feel free to use and modify as needed.

## Credits

Created to help Samira navigate the physical obstacles of her life more easily, with a casually elegant flick of her wrist. Batgirl-style!



