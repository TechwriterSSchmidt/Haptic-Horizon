# Haptic Horizon
The shin-saving, forehead-protecting sixth sense designed exclusively for Samira. Go batgirl mode! 🦇

## Overview
Haptic Horizon is a smart wearable **Electronic Travel Aid (ETA)** that fuses **LiDAR (ToF)**, **Thermal Imaging**, and **Motion Sensing** to translate the environment into a rich haptic language.

It behaves organically: **Lift it to scan, lower it to walk, drop it to sleep.**

Unlike simple proximity sensors, Haptic Horizon understands context:
*   **Smart Terrain:** Detects walls, drop-offs, and stairs using 3D gradient analysis.
*   **Heat Vision:** Identifies people, pets, and electronics by combining heat signatures with object size.
*   **Glass & Fog Detection:** Uses Ultrasonic waves to detect transparent obstacles (Glass) that LiDAR misses, and to filter out false positives from fog or steam.
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
- **Matrix Sensor**: VL53L8CX (Time-of-Flight 8x8 Multizone) - *Wide Angle Obstacle Detection*
- **Focus Sensor**: VL53L4CX (Time-of-Flight Single Point) - *Precise Aiming*
- **Ultrasonic Sensor**: GY-US42 (I2C) - *Glass & Transparent Obstacle Detection*
- **Thermal Camera**: MLX90640-BAB (32x24 Infrared Array)
- **IMU**: BMI160 (Gyroscope + Accelerometer)
- **Haptic Driver**: DRV2605L (Required - for advanced waveforms & LRA support)
- **Output**: LRA Vibration Motor (Connected to DRV2605L)
- **Input**: Single Mode Button (P0.29)
- **Power**: 3.7V LiPo Battery (SuperMini has built-in charging via B+/B- pads)

## Power Consumption & Battery Life
Estimated values based on a **2500mAh LiPo Battery**.

| Mode | Description | Avg. Current | Battery Life |
| :--- | :--- | :--- | :--- |
| **Scan Mode** | High Performance (All Sensors + Haptics) | ~170 mA | ~14-15 Hours |
| **Walk Mode** | Pathfinding (Matrix + Haptics) | ~140 mA | ~17-18 Hours |
| **Rest Mode** | Standby (Sensors On, Haptics Off) | ~110 mA | ~22 Hours |
| **Deep Sleep** | System OFF (Button Hold 2s) | < 1 mA | Months |

*Note: "Rest Mode" currently keeps sensors active for instant wake-up. For long storage, always turn the device OFF.*

## Power Supply (Important!)
*   **Recommended:** 3.7V LiPo/Li-Ion Battery.
    *   **Type:** 18650 Cell (2500mAh+) is excellent for a comfortable, grip-friendly handle design.
    *   Charges automatically via USB-C.
*   **Alternative:** Flat LiPo Pouch Cell (for slimmer designs).

## Pinout (SuperMini NRF52840)
| GPIO Pin | Function | Connected Component(s) |
| :--- | :--- | :--- |
| **P0.17** | I2C SDA (Primary) | VL53L4CX (Focus), BMI160, DRV2605L (Haptics), GY-US42 (Ultrasonic) |
| **P0.20** | I2C SCL (Primary) | VL53L4CX (Focus), BMI160, DRV2605L (Haptics), GY-US42 (Ultrasonic) |
| **P0.06** | I2C SDA (Secondary) | VL53L8CX (Matrix), MLX90640 (Thermal) |
| **P0.08** | I2C SCL (Secondary) | VL53L8CX (Matrix), MLX90640 (Thermal) |
| **P0.29** | Digital Input | Mode Button |
| **P0.02** | Analog Input | Battery Voltage Divider |
| **B+** | Power | LiPo Battery Positive |
| **B-** | Power | LiPo Battery Negative |

## User Guide

### 1. Power & Controls
*   **Power On/Off:** Press and **Hold for 2 seconds**.
    *   *On:* Plays Startup Melody.
    *   *Off:* Plays Shutdown Melody.
*   **Profile Switch:** Single Click.
    *   Toggles between **Indoor** (Sensitive, 2m) and **Outdoor** (Robust, 4m).
*   **Battery Check:** Double Click.
    *   Vibrates 1-4 times to indicate battery level.

### 2. Automatic Modes (IMU Controlled)
The device uses an internal gyroscope to switch modes automatically based on how you hold it.

*   **Zone 1: Scan Mode (Horizontal)**
    *   *Posture:* Hold up like a flashlight.
    *   *Function:* Precision scanning & **Heat Vision**. Detects door handles, narrow gaps, and **People** (Thermal Heartbeat).
    *   *Glass/Fog:* Uses Ultrasonic to detect glass doors (which LiDAR sees through) and ignore fog (which LiDAR reflects).
*   **Zone 2: Walk Mode (Diagonal)**
    *   *Posture:* Hold naturally at your side (~45° down).
    *   *Function:* Pathfinding (Matrix Sensor). Detects walls, furniture, and **Drop-offs**.
*   **Zone 3: Rest Mode (Vertical)**
    *   *Posture:* Let it hang on the strap.
    *   *Function:* **Standby.** Sensors paused.
    *   *Anti-Pendulum:* Requires 0.6s of stable holding to wake up from Rest Mode (prevents accidental scans while walking).
    *   *Auto-Calibration:* If held still for >2s, the sensors recalibrate automatically.
    *   *Auto-Off:* Turns off after 5 minutes.
*   **Zone 4: Pocket Mode (Bag/Pocket)**
    *   *Trigger:* Sensors covered (< 10cm).
    *   *Function:* **Muted.** Immediate silence. Turns off after 5 minutes.

### 3. Smart Features (The "Magic")
| Feature | Trigger | Action | Benefit |
| :--- | :--- | :--- | :--- |
| **Table Mute** | Device lies absolutely still for >3s. | **Mutes Haptics.** | Prevents annoying rattling on tables. |
| **Pocket Mode** | Sensors covered (Bag/Pocket). | **Mutes Haptics.** | Prevents false alarms while carrying it. |
| **Anti-Fog** | Laser sees "Wall" (<1m), Ultrasonic sees "Clear" (>2m). | **Ignores Laser.** | Prevents false alarms in fog/rain. |
| **Glass Alarm** | Laser sees "Clear" (>2m), Ultrasonic sees "Wall" (<1m). | **Sharp Tick.** | Warns of invisible glass doors. |
| **Drop Alarm** | Freefall detected (>2.5g). | **Loud Strobe.** | Helps you find the device if dropped. |
| **Overheat** | Internal Temp > 65°C. | **Triple Click.** | Warns before damage occurs. |
| **Low Battery** | Battery < 15%. | **Soft Bump (5min).** | Reminds you to charge soon. |

### 4. The Haptic Dictionary
| Signal Name | Pattern (Rhythm) | Feeling | Meaning |
| :--- | :--- | :--- | :--- |
| **Wall / Obstacle** | **Pulsed Buzz** | *Bzzz... Bzzz...* | Obstacle ahead. Faster = Closer. |
| **Drop-off** | **Ramp Down** | Falling Sensation | Ground disappears (Cliff/Stairs down). |
| **Glass / Detail** | **Sharp Tick** | *Tick... Tick...* | Precise object (Door handle, Pole). |
| **Glass Alarm** ("Batgirl-in-Danger") | **Sharp Tick** | *Tick... Tick...* | Invisible obstacle detected (Glass/Clear Wall). |
| **Human** | **Heartbeat** | *Bumm-Bumm...* | Person detected (Warmth). |
| **Profile: Indoor** | **Soft Bump** | *Thump* | Switched to Indoor Mode. |
| **Profile: Outdoor** | **Double Click** | *Click-Click* | Switched to Outdoor Mode. |
| **Overheat Warning** | **Triple Click** | *Click-Click-Click* | Internal Temp > 65°C. |
| **Overheat Shutdown** | **Long Buzz x2** | *BZZZ... BZZZ...* | Internal Temp > 75°C. |
| **Low Battery** | **Soft Bump** | *Thump* (every 5m) | Battery critical (<15%). |
| **Alarm** | **Loud Strobe** | *BZZZ-BZZZ* | "I am here!" (Find Me / Drop Alarm). |

### 5. Safety Features
*   **Drop Alarm:** If the device falls (>2.5g impact), it waits 5 seconds and then strobes loudly to help you find it.
*   **Overheat Protection:**
    *   **Warning (>65°C):** Triple Click (*Click-Click-Click*).
    *   **Shutdown (>75°C):** Emergency Shutdown (*Long Buzz x2*).
*   **Find Me:** Use a Bluetooth Camera Shutter (e.g., "AB Shutter3") to make the device buzz if you misplaced it.

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



