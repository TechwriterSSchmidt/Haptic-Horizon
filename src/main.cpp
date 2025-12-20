#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "config.h"
// Include Nordic GPIO HAL to bypass Arduino pin mapping and use P0.xx directly
#include <hal/nrf_gpio.h>
#include <nrf_soc.h> // For power management

SparkFun_VL53L5CX sensor;
VL53L5CX_ResultsData measurementData;

OperationMode currentMode = MODE_NAVIGATION;
unsigned long lastPulseTime = 0;
int pulseState = 0; 

// Power Management
unsigned long lastActivityTime = 0;

// Button Debouncing
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Forward declaration
void handleHapticsNavigation(int distance);
void handleHapticsPrecision(int distance);
void toggleMode();
void goToSleep();
#ifdef BUZZER_PIN
void playStartupMelody();
void playShutdownMelody();
#endif

void setup() {
  Serial.begin(115200);
  
  // Motor Setup
  nrf_gpio_cfg_output(MOTOR_PIN);
  nrf_gpio_pin_write(MOTOR_PIN, 0);

  // Button Setup
  nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);

  // Optional: Buzzer Setup
  #ifdef BUZZER_PIN
  pinMode(BUZZER_PIN, OUTPUT);
  playStartupMelody();
  #endif

  // I2C Setup
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_FREQUENCY); 

  Serial.println("Initializing VL53L5CX...");
  
  if (sensor.begin() == false) {
    Serial.println("Sensor not found - check wiring. Freezing");
    while (1);
  }

  sensor.setResolution(SENSOR_RESOLUTION); 
  sensor.setRangingFrequency(SENSOR_RANGING_FREQ);
  sensor.startRanging();

  Serial.println("Haptic Horizon Started");
  lastActivityTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // --- Auto-Off Check ---
  if (currentMillis - lastActivityTime > AUTO_OFF_MS) {
      goToSleep();
  }

  // --- Button Handling ---
  int reading = nrf_gpio_pin_read(BUTTON_PIN);
  
  // Simple state machine for button press (Active LOW)
  static int buttonState;
  static int lastReading = HIGH;
  
  if (reading != lastReading) {
      lastDebounceTime = currentMillis;
  }
  
  if ((currentMillis - lastDebounceTime) > debounceDelay) {
      if (reading != buttonState) {
          buttonState = reading;
          if (buttonState == LOW) {
              toggleMode();
              lastActivityTime = currentMillis; // Button press is activity
          }
      }
  }
  lastReading = reading;


  // --- Sensor Handling ---
  if (sensor.isDataReady()) {
    if (sensor.getRangingData(&measurementData)) {
      
      int targetDistance = 9999;

      if (currentMode == MODE_NAVIGATION) {
        // MODE 1: Safety Bubble (Check ALL zones)
        int validZones = 0;
        for (int i = 0; i < 16; i++) {
          if (measurementData.target_status[i] == 5 || measurementData.target_status[i] == 9) {
            int dist = measurementData.distance_mm[i];
            if (dist < targetDistance && dist > 0) targetDistance = dist;
            validZones++;
          }
        }
        if (validZones == 0) targetDistance = 9999;
        
        // Activity Check: If we are detecting something close, we are "active"
        if (targetDistance < DIST_FAR) {
            lastActivityTime = currentMillis;
        }
        
        handleHapticsNavigation(targetDistance);

      } else {
        // MODE 2: Precision Pointer (Check CENTER zones only)
        int validZones = 0;
        int centerZones[] = {5, 6, 9, 10};
        
        for (int i : centerZones) {
           if (measurementData.target_status[i] == 5 || measurementData.target_status[i] == 9) {
            int dist = measurementData.distance_mm[i];
            if (dist < targetDistance && dist > 0) targetDistance = dist;
            validZones++;
          }
        }
        if (validZones == 0) targetDistance = 9999;

        // Activity Check
        if (targetDistance < DIST_FAR) {
            lastActivityTime = currentMillis;
        }

        handleHapticsPrecision(targetDistance);
      }
      
      // Debug Output (Optional, can be removed for production)
      // Serial.print("Mode: ");
      // Serial.print(currentMode == MODE_NAVIGATION ? "NAV" : "PREC");
      // Serial.print(" | Dist: ");
      // Serial.println(targetDistance);
    }
  }
}

void goToSleep() {
    Serial.println("Auto-Off: Entering Deep Sleep.");
    
    #ifdef BUZZER_PIN
    playShutdownMelody();
    #else
    // Feedback: 3 short pulses to indicate shutdown if no buzzer
    for(int i=0; i<3; i++) {
        nrf_gpio_pin_write(MOTOR_PIN, 1); delay(100);
        nrf_gpio_pin_write(MOTOR_PIN, 0); delay(100);
    }
    #endif
    
    sensor.stopRanging();
    
    // Configure Button as Wakeup Source (Sense LOW level)
    nrf_gpio_cfg_sense_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    
    // Enter System OFF
    // Try SoftDevice Power Off first (if SD is enabled)
    sd_power_system_off();
    
    // Fallback for no SoftDevice
    NRF_POWER->SYSTEMOFF = 1;
    
    while(1); // Should not be reached
}

void toggleMode() {
    if (currentMode == MODE_NAVIGATION) {
        currentMode = MODE_PRECISION;
        
        #ifdef BUZZER_PIN
        // Sound: Zoom In (Ascending C5 -> E5)
        tone(BUZZER_PIN, 523, 100); delay(120);
        tone(BUZZER_PIN, 659, 100); delay(120);
        noTone(BUZZER_PIN);
        #endif

        // Feedback: Double Buzz
        nrf_gpio_pin_write(MOTOR_PIN, 1); delay(100);
        nrf_gpio_pin_write(MOTOR_PIN, 0); delay(100);
        nrf_gpio_pin_write(MOTOR_PIN, 1); delay(100);
        nrf_gpio_pin_write(MOTOR_PIN, 0);
    } else {
        currentMode = MODE_NAVIGATION;
        
        #ifdef BUZZER_PIN
        // Sound: Zoom Out (Descending E5 -> C5)
        tone(BUZZER_PIN, 659, 100); delay(120);
        tone(BUZZER_PIN, 523, 100); delay(120);
        noTone(BUZZER_PIN);
        #endif

        // Feedback: Single Long Buzz
        nrf_gpio_pin_write(MOTOR_PIN, 1); delay(300);
        nrf_gpio_pin_write(MOTOR_PIN, 0);
    }
}

void handleHapticsNavigation(int distance) {
  unsigned long currentMillis = millis();
  int pulseInterval = 0;

  if (distance > DIST_FAR) {
    nrf_gpio_pin_write(MOTOR_PIN, 0);
    pulseState = 0;
    return;
  } 
  else if (distance > DIST_MEDIUM) {
    pulseInterval = 600; 
  } 
  else if (distance > DIST_CLOSE) {
    pulseInterval = map(distance, DIST_CLOSE, DIST_MEDIUM, 100, 600);
  } 
  else {
    nrf_gpio_pin_write(MOTOR_PIN, 1);
    pulseState = 1;
    return; 
  }

  if (pulseInterval > 0) {
    if (currentMillis - lastPulseTime >= pulseInterval) {
      lastPulseTime = currentMillis;
      if (pulseState == 0) {
        pulseState = 1;
        nrf_gpio_pin_write(MOTOR_PIN, 1);
      } else {
        pulseState = 0;
        nrf_gpio_pin_write(MOTOR_PIN, 0);
      }
    }
  }
}

void handleHapticsPrecision(int distance) {
    // Analog-like feedback using PWM (simulated via fast switching or real PWM if configured)
    // For simplicity with GPIO HAL, we'll use a "Geiger Counter" style clicking
    // The closer, the faster the clicks.
    
    unsigned long currentMillis = millis();
    int clickInterval = 0;

    if (distance > 2000) {
        // Silence
        nrf_gpio_pin_write(MOTOR_PIN, 0);
        return;
    }
    
    // Map distance to click speed
    // 10cm = 20ms interval (buzzing)
    // 200cm = 500ms interval (slow clicking)
    clickInterval = map(distance, 100, 2000, 20, 500);
    clickInterval = constrain(clickInterval, 20, 1000);

    // Short blip
    if (currentMillis - lastPulseTime >= clickInterval) {
        lastPulseTime = currentMillis;
        // Trigger a very short pulse (blocking is okay for 5ms here)
        nrf_gpio_pin_write(MOTOR_PIN, 1);
        delay(5); 
        nrf_gpio_pin_write(MOTOR_PIN, 0);
    }
}

#ifdef BUZZER_PIN
void playStartupMelody() {
    // La Marseillaise Opening (Extended)
    // D4, D4, D4, G4, G4, A4, A4, D5 ... B4, B4
    // "Al-lons en-fants de la Pa-trie... Le jour..."
    
    int melody[] = { 294, 294, 294, 392, 392, 440, 440, 587, 494, 494 };
    int durations[] = { 150, 150, 150, 400, 400, 400, 400, 800, 300, 300 };
    
    for (int i = 0; i < 10; i++) {
        tone(BUZZER_PIN, melody[i], durations[i]);
        delay(durations[i] * 1.30); // Wait for note to finish + gap
        noTone(BUZZER_PIN);
    }
}

void playShutdownMelody() {
    // Classic Windows Shutdown Style
    // Eb5, Bb4, F4
    
    int melody[] = { 622, 466, 349 };
    int durations[] = { 250, 250, 800 };
    
    for (int i = 0; i < 3; i++) {
        tone(BUZZER_PIN, melody[i], durations[i]);
        delay(durations[i] * 1.10);
        noTone(BUZZER_PIN);
    }
}
#endif
