#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "config.h"
// Include Nordic GPIO HAL to bypass Arduino pin mapping and use P0.xx directly
#include <hal/nrf_gpio.h>
#include <nrf_soc.h> // For power management
#include <bluefruit.h> // Bluetooth LE

SparkFun_VL53L5CX sensor;
VL53L5CX_ResultsData measurementData;

// Bluetooth Services
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // Device Information
BLEUart bleuart; // UART over BLE

OperationMode currentMode = MODE_NAVIGATION;
unsigned long lastPulseTime = 0;
int pulseState = 0; 

// Power Management
unsigned long lastActivityTime = 0;
bool isStandby = false; // New State for "Find Me" mode

// Button Debouncing
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Forward declaration
void handleHapticsNavigation(int distance);
void handleHapticsPrecision(int distance);
void toggleMode();
void enterStandby();
void wakeUp();
#ifdef BUZZER_PIN
void playStartupMelody();
void playShutdownMelody();
#endif

#ifdef ENABLE_SELFIE_FINDER
void scan_callback(ble_gap_evt_adv_report_t* report);
#endif

void setup() {
  Serial.begin(115200);
  // Give Serial some time to start
  delay(100);
  
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

  // BLE Setup
  // Note: 1 connection for phone, 1 for potential central role if needed
  Bluefruit.begin(1, 1); 
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Haptic Horizon");

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Haptic Horizon Team");
  bledis.setModel("V1.0");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Set up Advertising Packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  
  // Start Advertising
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  

  #ifdef ENABLE_SELFIE_FINDER
  // Configure Scanner
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // Internal scanning interval (not the duty cycle)
  Bluefruit.Scanner.useActiveScan(true);  // Request scan response data
  // We don't start scanning yet, only in Standby
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

  // --- Watchdog Setup ---
  // Pause WDT when CPU is sleeping (to allow long Standby/BLE waits)
  NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Pause << WDT_CONFIG_SLEEP_Pos) | (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);
  NRF_WDT->CRV = 32768 * WATCHDOG_TIMEOUT_SEC; // Configurable Timeout (32768 Hz clock)
  NRF_WDT->RREN = WDT_RREN_RR0_Msk; // Enable Reload Register 0
  NRF_WDT->TASKS_START = 1; // Start WDT
}

void loop() {
  // Feed the Watchdog
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;

  // --- BLE Handling (Find Me) ---
  // If connected via BLE, check for commands
  if (Bluefruit.connected() && bleuart.notifyEnabled()) {
    while (bleuart.available()) {
      uint8_t ch = (uint8_t) bleuart.read();
      // If we receive 'B' (Beep) or 'F' (Find), play melody
      if (ch == 'B' || ch == 'F' || ch == 'b' || ch == 'f') {
          #ifdef BUZZER_PIN
          // Play Marseillaise (Loop for visibility)
          for(int i=0; i<5; i++) {
            playStartupMelody();
            delay(500);
          }
          #endif
          // Reset activity timer so we don't sleep immediately
          lastActivityTime = millis();
          if (isStandby) wakeUp();
      }
    }
  }

  // --- Standby Mode Handling ---
  if (isStandby) {
      // In Standby, we just wait for Button or BLE
      // Blink LED or sleep CPU lightly could go here
      
      // Check Button to Wake Up
      int reading = nrf_gpio_pin_read(BUTTON_PIN);
      if (reading == LOW) { // Button Pressed
          wakeUp();
      }
      
      // Low Power Wait (System ON, waiting for interrupts/events)
      sd_app_evt_wait(); 
      return; // Skip the rest of the loop
  }

  unsigned long currentMillis = millis();

  // --- Auto-Off Check ---
  if (currentMillis - lastActivityTime > AUTO_OFF_MS) {
      enterStandby();
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

void enterStandby() {
    Serial.println("Auto-Off: Entering BLE Standby.");
    
    #ifdef BUZZER_PIN
    playShutdownMelody();
    #endif
    
    sensor.stopRanging();
    isStandby = true;
    
    // Slow down advertising to save power
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.setInterval(3200, 3200); // 2 seconds interval
    Bluefruit.Advertising.start(0);

    #ifdef ENABLE_SELFIE_FINDER
    // Start scanning with low duty cycle
    // Interval: 4000ms (0x1900 units of 0.625ms = 6400)
    // Window: 200ms (0xC8 units of 0.625ms = 320)
    // Note: Bluefruit.Scanner.setInterval(window, interval)
    Bluefruit.Scanner.setInterval(SCAN_WINDOW_MS/0.625, SCAN_INTERVAL_MS/0.625);
    Bluefruit.Scanner.start(0);
    Serial.println("Scanner started (Low Duty Cycle)");
    #endif
}

void wakeUp() {
    Serial.println("Waking Up!");
    isStandby = false;
    lastActivityTime = millis();
    
    #ifdef ENABLE_SELFIE_FINDER
    Bluefruit.Scanner.stop();
    #endif

    // Restart Sensor
    sensor.startRanging();
    
    // Speed up advertising
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.start(0);
    
    #ifdef BUZZER_PIN
    playStartupMelody();
    #endif
}

#ifdef ENABLE_SELFIE_FINDER
void scan_callback(ble_gap_evt_adv_report_t* report) {
  // Check if the device name matches our target
  // Note: We need to parse the advertising packet to find the name
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));
  
  // Check for Complete Local Name (0x09) or Shortened Local Name (0x08)
  if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)) ||
      Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, buffer, sizeof(buffer))) 
  {
      const char* name = (const char*)buffer;
      // Serial.print("Found: "); Serial.println(name);
      
      if (strstr(name, SELFIE_BUTTON_NAME) != NULL || strstr(name, "Remote") != NULL || strstr(name, "Shutter") != NULL) {
          Serial.println("FOUND SELFIE BUTTON! TRIGGERING ALARM!");
          
          #ifdef BUZZER_PIN
          // Play Marseillaise (Loop for ~20 seconds)
          for(int loop=0; loop<5; loop++) {
             playStartupMelody();
             delay(500);
          }
          #endif
          
          // Wake up fully after alarm
          wakeUp();
      }
  }
}
#endif

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
