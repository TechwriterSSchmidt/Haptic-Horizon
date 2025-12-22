# Quick Reference Guide: Haptic Horizon

*Your silent sixth sense for navigation. Simple, intuitive, and discreet.*

> **Concept:** This device translates the physical world into vibration patterns ("Hapticons"). It behaves organically: lift it to scan, lower it to walk, drop it to sleep.

---

## 1. The Haptic Dictionary (What you feel)

| Signal Name | Pattern (Rhythm) | Feeling | Meaning | Action |
| :--- | :--- | :--- | :--- | :--- |
| **Wall / Obstacle** | **Pulsed Buzz** | *Bzzz... Bzzz...* | Obstacle ahead. Faster = Closer. | Stop or go around. |
| **Drop-off** | **Ramp Down** | Falling Sensation | Ground disappears (Cliff/Stairs down). | **STOP immediately!** |
| **Glass / Detail** | **Sharp Tick** | *Tick... Tick...* | Precise object (Door handle, Pole). | Proceed with caution. |
| **Human** | **Heartbeat** | *Bumm-Bumm...* | Person detected (Warmth). | - |
| **Profile: Indoor** | **Soft Bump** | *Thump* | Switched to Indoor Mode (Short Range). | - |
| **Profile: Outdoor** | **Double Click** | *Click-Click* | Switched to Outdoor Mode (Long Range). | - |
| **Battery Check** | **Pulses** | *Tick-Tick...* | 4=Full, 3=Good, 2=Low, 1=Critical. | Charge if low. |
| **Glass Alarm** ("Batgirl-in-Danger") | **Sharp Tick** | *Tick... Tick...* | Invisible obstacle detected (Glass/Clear Wall). | **STOP!** Do not walk through. |
| **Hot Surface (Danger)** | **Fast Strobe** | *Panic Pulse* | External Heat Source > 60°C. | **STOP!** Hot surface nearby. |
| **Overheat Warning** | **Triple Click** | *Click-Click-Click* | Internal Temp > 65°C. | Turn off & Cool down. |
| **Overheat Shutdown** | **Long Buzz x2** | *BZZZ... BZZZ...* | Internal Temp > 75°C. | **Device turns OFF.** |

---

## 2. Controls (Single Button)

### Power
*   **Turn On/Off:** Press and **Hold for 2 seconds**.
    *   *On:* Plays Startup Melody.
    *   *Off:* Plays Shutdown Melody.

### Profiles (Short Click)
*   **Action:** Single Click.
*   **Function:** Toggles between **Indoor** (2m Range, Sensitive) and **Outdoor** (4m Range, Robust).

### Battery Check (Double Click)
*   **Action:** Double Click.
*   **Function:** Vibrates 1-4 times to indicate battery level.

---

## 3. Automatic Modes (How to hold it)

The device uses an internal gyroscope to switch modes automatically based on how you hold it.

### Zone 1: Scan Mode (Horizontal)
*   **Posture:** Hold the device up, pointing forward (like a flashlight).
*   **Function:** Precision scanning, **Heat Vision**, and **Glass Detection** ("Batgirl-in-Danger").
*   **Sensors:** 
    *   **Focus Sensor (Laser):** Detects small objects.
    *   **Thermal Camera:** Detects people (Heartbeat feedback).
    *   **Ultrasonic:** Detects glass and transparent surfaces.
    *   **Anti-Fog (Outdoor Only):** If the Laser sees "Fog" (Close) but Ultrasonic sees "Clear" (Far), the device ignores the false alarm.

### Zone 2: Walk Mode (Diagonal)
*   **Posture:** Hold the device naturally at your side, pointing down at the floor (~45°).
*   **Function:** Pathfinding. Detects walls, furniture, and **Drop-offs** (Stairs/Holes).
*   **Sensor:** Uses the **Matrix Sensor** (Wide Angle).

### Zone 3: Rest Mode (Vertical)
*   **Posture:** Let the device hang straight down on its strap.
*   **Function:** **Standby.** Sensors and motors are paused to save battery and silence the device.
*   **Anti-Pendulum:** The device will not wake up if it swings momentarily. You must hold it steady in a scanning position for **0.6 seconds** to wake it up.
*   **Auto-Off:** If left in Rest Mode for **5 minutes**, the device turns off completely.

### Zone 4: Pocket Mode (Bag/Pocket)
*   **Trigger:** Putting the device in a bag or pocket (Sensors covered < 10cm).
*   **Function:** **Muted.** Immediate silence (no vibration) to prevent accidental feedback.
*   **Auto-Off:** Turns off completely after **5 minutes** to save battery.

---

## 4. Safety Features

### Drop Alarm
If you drop the device, it will detect the freefall and impact. After 5 seconds of silence (to let you find it), it will start buzzing loudly so you can locate it by sound/vibration.

### Table Mute
If the device is placed on a table or stable surface (absolutely still for > 3 seconds), it will automatically mute all vibrations to prevent noise. It wakes up instantly when picked up.

---

## 5. Limitations & Care (IMPORTANT)

### ⚠️ Weather & Water
*   **Not Waterproof:** The sensors (Laser, Thermal, Ultrasonic) have open lenses/grills.
*   **Rain Risk:** Do **NOT** use the device in rain, heavy mist, or snow. Water droplets on the lenses will cause false alarms or permanent damage to the electronics.
*   **Action:** If it starts raining, put the device in your bag immediately. If it gets wet, turn it off and let it dry completely before using it again.

### ⚠️ Glass & Mirrors
*   **Laser Limitations:** The main laser sensors may look *through* clean glass.
*   **Solution:** The device uses a backup **Ultrasonic Sensor** in "Scan Mode" to detect glass. Always scan horizontally if you suspect a glass door.

### ⚠️ Black Surfaces
*   **Absorption:** Very black, shiny floors or objects may absorb the laser light, making them invisible to the main sensors.
*   **Safety:** Use the "Walk Mode" carefully on unknown black surfaces.
