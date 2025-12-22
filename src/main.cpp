#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include "config.h"
#include <math.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <hal/nrf_gpio.h>
#include <nrf_soc.h>
#include <bluefruit.h>

// --- NEW CLASSES ---
#include "SmartTerrain.h"
#include "HapticEngine.h"

using namespace Adafruit_LittleFS_Namespace;

// Objects
DFRobot_BMI160 bmi160;
SmartTerrain terrain;
HapticEngine haptics;

// Define Secondary I2C Bus (Wire1) using TWIM1
TwoWire Wire1(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, SDA1_PIN, SCL1_PIN);

// Bluetooth Services
BLEDfu  bledfu;
BLEDis  bledis;
BLEUart bleuart;

// Drop Beacon State
bool dropDetected = false;
unsigned long dropTime = 0;
bool dropAlarmActive = false;

// Calibration
float pitchOffset = MOUNTING_PITCH_OFFSET;

// Power Management
unsigned long lastActivityTime = 0;
unsigned long lastTempCheck = 0;

// Forward declaration
void goToSleep();
void calibrateIMU();
void checkForDrop(int16_t* accelGyro);
void announceBatteryLevel();
void checkInternalTemperature();

#ifdef ENABLE_SELFIE_FINDER
void scan_callback(ble_gap_evt_adv_report_t* report);
#endif

void setup() {
  Serial.begin(115200);
  delay(100);

  // Button Setup
  nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);

  // --- WAKEUP CHECK (Enforce 2s Hold) ---
  // Only check if button is pressed (Wake from Sleep)
  if (nrf_gpio_pin_read(BUTTON_PIN) == 0) {
      unsigned long startHold = millis();
      bool validWake = false;
      while (nrf_gpio_pin_read(BUTTON_PIN) == 0) {
          if (millis() - startHold > POWER_PRESS_MS) {
              validWake = true;
              break; 
          }
          delay(10);
      }
      
      if (!validWake) {
          // Go back to sleep silently (Button released too early)
          nrf_gpio_cfg_sense_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
          NRF_POWER->SYSTEMOFF = 1;
          while(1);
      }
  }

  // BLE Setup
  Bluefruit.begin(1, 1); 
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Haptic Horizon");
  bledfu.begin();
  bledis.setManufacturer("Haptic Horizon Team");
  bledis.setModel("V1.0");
  bledis.begin();
  bleuart.begin();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  #ifdef ENABLE_SELFIE_FINDER
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.useActiveScan(true);
  #endif

  // I2C Setup
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_FREQUENCY); 
  Wire1.setPins(SDA1_PIN, SCL1_PIN);
  Wire1.begin();
  Wire1.setClock(I2C_FREQUENCY);

  // Init Haptics
  Serial.println("Initializing Haptics...");
  haptics.begin(&Wire, &Wire1);
  haptics.playStartup();

  // Init File System
  if (InternalFS.begin()) {
      if (InternalFS.exists("/calib.txt")) {
          Adafruit_LittleFS_Namespace::File file = InternalFS.open("/calib.txt", FILE_O_READ);
          if (file) {
              pitchOffset = file.parseFloat();
              file.close();
          }
      }
  }

  // Init BMI160
  if (bmi160.softReset() != BMI160_OK) Serial.println("BMI160 Reset Failed");
  if (bmi160.I2cInit(BMI160_I2C_ADDR) != BMI160_OK) Serial.println("BMI160 Init Failed");

  // Init Smart Terrain (Sensors)
  Serial.println("Initializing Sensors...");
  terrain.begin(&Wire, &Wire1);

  Serial.println("Haptic Horizon Started");
  lastActivityTime = millis();

  // Watchdog
  NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Pause << WDT_CONFIG_SLEEP_Pos) | (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);
  NRF_WDT->CRV = 32768 * WATCHDOG_TIMEOUT_SEC;
  NRF_WDT->RREN = WDT_RREN_RR0_Msk;
  NRF_WDT->TASKS_START = 1;
}

void loop() {
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;

  // --- BLE Handling ---
  if (Bluefruit.connected() && bleuart.notifyEnabled()) {
    while (bleuart.available()) {
      uint8_t ch = (uint8_t) bleuart.read();
      if (ch == 'B' || ch == 'F' || ch == 'b' || ch == 'f') {
          haptics.playFindMe();
          lastActivityTime = millis();
      }
    }
  }

  unsigned long currentMillis = millis();

  // --- Temperature Check ---
  if (currentMillis - lastTempCheck > TEMP_CHECK_INTERVAL_MS) {
      lastTempCheck = currentMillis;
      checkInternalTemperature();
  }

  // --- IMU Handling ---
  int16_t accelGyro[6]={0};
  float pitch = 0;
  
  if (bmi160.getAccelGyroData(accelGyro) == BMI160_OK) {
      int gyroActivity = abs(accelGyro[0]) + abs(accelGyro[1]) + abs(accelGyro[2]);
      if (gyroActivity > 200) lastActivityTime = currentMillis;

      float accelY = accelGyro[4];
      float accelZ = accelGyro[5];
      pitch = (atan2(accelY, accelZ) * 180.0 / PI) + pitchOffset;
      
      checkForDrop(accelGyro);
  }

  if (currentMillis - lastActivityTime > AUTO_OFF_MS) goToSleep();

  // --- Button Handling (Single Button Logic) ---
  static bool buttonActive = false;
  static unsigned long buttonPressStartTime = 0;
  static int clickCount = 0;
  static unsigned long lastReleaseTime = 0;
  static bool longPressHandled = false;

  int reading = nrf_gpio_pin_read(BUTTON_PIN);

  if (reading == 0) { // Pressed
      if (!buttonActive) {
          buttonActive = true;
          buttonPressStartTime = currentMillis;
          longPressHandled = false;
      } else {
          // Check for Long Press (Power Off)
          if (!longPressHandled && (currentMillis - buttonPressStartTime > POWER_PRESS_MS)) {
              longPressHandled = true;
              goToSleep();
          }
      }
  } else { // Released
      if (buttonActive) {
          buttonActive = false;
          unsigned long duration = currentMillis - buttonPressStartTime;
          
          if (duration > 50 && !longPressHandled) { // Valid short press
              clickCount++;
              lastReleaseTime = currentMillis;
          }
      }
  }

  // Process Clicks (Delayed to detect double clicks)
  if (clickCount > 0 && (currentMillis - lastReleaseTime > 400)) {
      if (clickCount == 1) {
          // Single Click: Toggle Profile
          terrain.toggleProfile();
          if (terrain.isOutdoorProfile()) {
              haptics.playEffect(EFFECT_DOUBLE_CLICK, 0); // Outdoor: Two clicks
          } else {
              haptics.playEffect(EFFECT_SOFT_BUMP, 0); // Indoor: Soft bump
          }
      } else if (clickCount >= 2) {
          // Double Click: Battery Level
          announceBatteryLevel();
      }
      clickCount = 0;
      lastActivityTime = currentMillis;
  }


  // --- SENSOR FUSION & LOGIC ---
  terrain.updateThermal();
  terrain.update(pitch);

  // --- AUTO-OFF LOGIC (Rest Mode / Handbag) ---
  static unsigned long restModeStartTime = 0;
  
  if (terrain.isInRestMode() || terrain.isBlocked()) {
      if (restModeStartTime == 0) restModeStartTime = currentMillis;
      
      if (currentMillis - restModeStartTime > AUTO_OFF_REST_MS) {
          Serial.println("Auto-Off: Rest Mode Timeout");
          goToSleep();
      }
  } else {
      restModeStartTime = 0; // Reset timer if active
  }

  // --- HAPTIC FEEDBACK ---
  int mode = terrain.getHapticMode();
  int interval = terrain.getHapticInterval();
  int dir = terrain.getWallDirection();
  
  // Map SmartTerrain modes to HapticEngine effects
  haptics.playPattern(mode, 255, interval, dir);
}

void goToSleep() {
    Serial.println("Going to System OFF...");
    delay(100);
    haptics.playShutdown();
    terrain.stop();
    
    // Wake up on Button Press (Low)
    nrf_gpio_cfg_sense_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    
    NRF_POWER->SYSTEMOFF = 1;
    while(1);
}

#ifdef ENABLE_SELFIE_FINDER
void scan_callback(ble_gap_evt_adv_report_t* report) {
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));
  if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)) ||
      Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, buffer, sizeof(buffer))) 
  {
      const char* name = (const char*)buffer;
      if (strstr(name, SELFIE_BUTTON_NAME) != NULL || strstr(name, "Remote") != NULL || strstr(name, "Shutter") != NULL) {
          haptics.playEffect(EFFECT_PULSING_SHARP, 0);
          lastActivityTime = millis();
      }
  }
}
#endif

void announceBatteryLevel() {
    // 1. Silence everything for accurate reading
    haptics.stop();
    terrain.stop(); // Stop ToF ranging to reduce current spikes
    delay(100); // Wait for voltage to stabilize

    // 2. Read Voltage
    int raw = analogRead(BATTERY_PIN);
    float voltage = (raw / 1023.0) * 3.3 * 2.0; 

    // 3. Play Feedback
    haptics.playBatteryLevel(voltage);

    // 4. Restart Sensors
    terrain.start();
}

void calibrateIMU() {
    // Calibration logic moved to specific button combo if needed, 
    // or we can keep it on a very long press (e.g. 10s) if desired.
    // For now, removing the 10s hold from the main loop to simplify.
    // If needed, we can add it back as a triple click or >10s hold.
    
    Serial.println("Calibrating IMU...");
    int16_t accelGyro[6]={0};
    float sumPitch = 0;
    int samples = 0;
    for(int i=0; i<10; i++) {
        if (bmi160.getAccelGyroData(accelGyro) == BMI160_OK) {
            float accelY = accelGyro[4];
            float accelZ = accelGyro[5];
            sumPitch += atan2(accelY, accelZ) * 180.0 / PI;
            samples++;
        }
        delay(20);
    }
    if (samples > 0) {
        pitchOffset = -(sumPitch / samples);
        InternalFS.remove("/calib.txt");
        Adafruit_LittleFS_Namespace::File file = InternalFS.open("/calib.txt", FILE_O_WRITE);
        if (file) {
            file.print(pitchOffset);
            file.close();
            haptics.playCalibrationSuccess();
        }
    }
}

void checkForDrop(int16_t* accelGyro) {
    long x = accelGyro[3]; long y = accelGyro[4]; long z = accelGyro[5];
    long magSq = x*x + y*y + z*z;
    
    if (magSq > 1600000000) { // ~2.5g squared
        if (!dropDetected) {
            dropDetected = true;
            dropTime = millis();
        }
    }
    
    if (dropDetected) {
        unsigned long timeSinceDrop = millis() - dropTime;
        if (timeSinceDrop > DROP_WAIT_TIME_MS && timeSinceDrop < (DROP_WAIT_TIME_MS + DROP_ALARM_DURATION_MS)) {
            if (!dropAlarmActive) dropAlarmActive = true;
            if (millis() % 1000 < 200) haptics.playEffect(EFFECT_BUZZ, 0);
        } else if (timeSinceDrop > (DROP_WAIT_TIME_MS + DROP_ALARM_DURATION_MS)) {
            dropDetected = false;
            dropAlarmActive = false;
        }
    }
}

void checkInternalTemperature() {
    // Read BMI160 Temperature (0x20, 0x21)
    Wire.beginTransmission(BMI160_I2C_ADDR);
    Wire.write(0x20);
    Wire.endTransmission();
    
    Wire.requestFrom((uint8_t)BMI160_I2C_ADDR, (uint8_t)2);
    if (Wire.available() == 2) {
        uint8_t lsb = Wire.read();
        uint8_t msb = Wire.read();
        int16_t rawTemp = (int16_t)((msb << 8) | lsb);
        
        // Formula from BMI160 Datasheet: 23°C + (raw * 0.001953°C)
        // 0x0000 is 23°C.
        float tempC = 23.0 + (rawTemp * 0.001953);
        
        // Debug output (optional, can be commented out)
        // Serial.print("Internal Temp: "); Serial.println(tempC);
        
        if (tempC > TEMP_CRITICAL_THRESHOLD) {
            Serial.print("CRITICAL: Overheat Shutdown! Temp: "); Serial.println(tempC);
            // Emergency Shutdown Sequence (Long Buzz x2)
            haptics.playEffect(EFFECT_BUZZ, 0);
            delay(500);
            haptics.playEffect(EFFECT_BUZZ, 0);
            delay(500);
            goToSleep();
        } else if (tempC > TEMP_WARNING_THRESHOLD) {
            Serial.print("WARNING: Overheat! Temp: "); Serial.println(tempC);
            // Trigger Haptic Warning (3 short pulses)
            haptics.playEffect(EFFECT_DOUBLE_CLICK, 0); 
            delay(200);
            haptics.playEffect(EFFECT_DOUBLE_CLICK, 0);
            delay(200);
            haptics.playEffect(EFFECT_DOUBLE_CLICK, 0);
        }
    }
}

