#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <DFRobot_BMI160.h>
#include "config.h"
#include <math.h> // Required for Smart Terrain trigonometry
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Adafruit_AMG88xx.h>

#ifdef ENABLE_DRV2605
#include <Adafruit_DRV2605.h>
#endif

using namespace Adafruit_LittleFS_Namespace;

// Include Nordic GPIO HAL to bypass Arduino pin mapping and use P0.xx directly
#include <hal/nrf_gpio.h>
#include <nrf_soc.h> // For power management
#include <bluefruit.h> // Bluetooth LE

SparkFun_VL53L5CX sensor;
VL53L5CX_ResultsData measurementData;
DFRobot_BMI160 bmi160;
Adafruit_AMG88xx amg;

#ifdef ENABLE_DRV2605
Adafruit_DRV2605 drv;
#endif

// Bluetooth Services
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // Device Information
BLEUart bleuart; // UART over BLE

OperationMode currentMode = MODE_SMART_TERRAIN;
OperationMode previousMode = MODE_SMART_TERRAIN;
// bool isDropOffActive = false; // Deprecated in favor of continuous Smart Terrain logic

// Drop Beacon State
bool dropDetected = false;
unsigned long dropTime = 0;
bool dropAlarmActive = false;
float amgPixels[AMG88xx_PIXEL_ARRAY_SIZE];

// Calibration
float pitchOffset = MOUNTING_PITCH_OFFSET; // Loaded from file if available

unsigned long lastPulseTime = 0;
int pulseState = 0; 

// Power Management
unsigned long lastActivityTime = 0;
// bool isStandby = false; // Removed: We use System OFF now

// Button Debouncing
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
bool calibrationTriggered = false; // Flag to prevent double action on release

// Forward declaration
// void handleHapticsNavigation(int distance); // Removed
void handleHapticsPrecision(int distance);
void toggleMode();
void goToSleep(); // Replaces enterStandby
// void wakeUp(); // Removed: Reset handles wakeup
void calibrateIMU();
void checkForDrop(int16_t* accelGyro);
void runHeatVision();
#ifdef BUZZER_PIN
void playStartupMelody();
void playShutdownMelody();
void playCalibrationSuccess();
void playTone(unsigned int frequency, unsigned long duration, int volume);
void announceBatteryLevel();
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
  nrf_gpio_cfg_input(TRIGGER_PIN, NRF_GPIO_PIN_PULLUP);

  // --- DOUBLE TAP WAKEUP CHECK ---
  // If we woke up from System OFF via Button, the button is likely still pressed.
  if (nrf_gpio_pin_read(BUTTON_PIN) == 0 || nrf_gpio_pin_read(TRIGGER_PIN) == 0) {
      // 1. Wait for Release (Debounce)
      unsigned long start = millis();
      while ((nrf_gpio_pin_read(BUTTON_PIN) == 0 || nrf_gpio_pin_read(TRIGGER_PIN) == 0) && millis() - start < 500) {
          delay(10);
      }
      
      // 2. Wait for Second Tap (1000ms Window)
      bool secondTap = false;
      start = millis();
      while (millis() - start < 1000) {
          if (nrf_gpio_pin_read(BUTTON_PIN) == 0 || nrf_gpio_pin_read(TRIGGER_PIN) == 0) {
              secondTap = true;
              break;
          }
          delay(10);
      }
      
      if (!secondTap) {
          // No second tap -> Go back to sleep
          goToSleep();
      }
  }
  // If no button pressed (Battery Insert), we boot normally.

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

  #ifdef ENABLE_DRV2605
  Serial.println("Initializing DRV2605L...");
  if (!drv.begin()) {
    Serial.println("DRV2605L not found! Check wiring.");
    // Don't freeze, maybe just fallback? But for now let's warn.
  } else {
    drv.selectLibrary(1); // Library 1 = Strong Click (ERM) / Sharp Click (LRA)
    
    // I2C trigger by default
    drv.setMode(DRV2605_MODE_INTTRIG); 
    
    #ifdef DRV2605_MOTOR_TYPE_LRA
    drv.useLRA();
    Serial.println("DRV2605L Configured for LRA");
    #else
    drv.useERM();
    Serial.println("DRV2605L Configured for ERM");
    #endif

    // Voltage Configuration for Motor Protection
    // DRV2605L Datasheet: 
    // Rated Voltage (0x16) = V_rated / 21.18mV
    // Overdrive Clamp (0x17) = V_clamp / 21.59mV
    uint8_t rated_val = (uint8_t)(MOTOR_RATED_VOLTAGE / 0.02118);
    uint8_t clamp_val = (uint8_t)(MOTOR_CLAMP_VOLTAGE / 0.02159);
    
    drv.writeRegister8(DRV2605_REG_RATEDV, rated_val);
    drv.writeRegister8(DRV2605_REG_CLAMPV, clamp_val);
    
    Serial.print("DRV2605L Voltage Limit set to: ");
    Serial.print(MOTOR_RATED_VOLTAGE);
    Serial.println("V");
  }
  #endif

  // Init File System for Calibration
  if (InternalFS.begin()) {
      if (InternalFS.exists("/calib.txt")) {
          File file = InternalFS.open("/calib.txt", FILE_O_READ);
          if (file) {
              float storedOffset = file.parseFloat();
              pitchOffset = storedOffset;
              file.close();
              Serial.print("Loaded Calibration: "); Serial.println(pitchOffset);
          }
      }
  } else {
      Serial.println("FS Init Failed");
  }

  // Init BMI160
  if (bmi160.softReset() != BMI160_OK) {
    Serial.println("BMI160 Reset Failed");
  }
  if (bmi160.I2cInit(BMI160_I2C_ADDR) != BMI160_OK) {
    Serial.println("BMI160 Init Failed");
  }

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
      }
    }
  }

  unsigned long currentMillis = millis();

  // --- IMU Handling (Motion & Tilt) ---
  int16_t accelGyro[6]={0};
  float pitch = 0;
  
  // Get raw data (Gyro: 0-2, Accel: 3-5)
  if (bmi160.getAccelGyroData(accelGyro) == BMI160_OK) {
      // 1. Auto-Off Logic (Motion Detection)
      int gyroActivity = abs(accelGyro[0]) + abs(accelGyro[1]) + abs(accelGyro[2]);
      if (gyroActivity > 200) { 
          lastActivityTime = currentMillis;
      }

      // 2. Calculate Pitch (Tilt)
      // Accel Y is forward/backward tilt usually. Z is up.
      float accelY = accelGyro[4];
      float accelZ = accelGyro[5];
      pitch = (atan2(accelY, accelZ) * 180.0 / PI) + pitchOffset;
      
      // 3. Check for Drop
      checkForDrop(accelGyro);
  }

  // --- Auto-Off Check ---
  if (currentMillis - lastActivityTime > AUTO_OFF_MS) {
      goToSleep();
  }

  // --- Button Handling ---
  int reading = nrf_gpio_pin_read(BUTTON_PIN);
  static bool buttonActive = false;
  static unsigned long buttonPressStartTime = 0;

  if (reading == 0) { // Pressed (LOW)
      if (!buttonActive) {
          buttonActive = true;
          buttonPressStartTime = currentMillis;
          calibrationTriggered = false;
      } else {
          // Check for Long Press (Calibration) > 10s
          if (!calibrationTriggered && (currentMillis - buttonPressStartTime > 10000)) {
              calibrateIMU();
              calibrationTriggered = true; // Prevent re-triggering
          }
      }
  } else { // Released
      if (buttonActive) {
          unsigned long duration = currentMillis - buttonPressStartTime;
          buttonActive = false;
          
          if (calibrationTriggered) {
              // Do nothing, action already handled
          }
          else if (duration > 50) { // Debounce
              lastActivityTime = currentMillis;
              
              if (duration > 2000) {
                  // Long Press (> 2s) -> Battery
                  #ifdef BUZZER_PIN
                  announceBatteryLevel();
                  #endif
              } else {
                  // Short Press -> Toggle Mode
                  toggleMode();
              }
          }
      }
  }

  // --- Trigger Handling (Heat Vision) ---
  int triggerReading = nrf_gpio_pin_read(TRIGGER_PIN);
  if (triggerReading == 0) { // Pressed
      if (currentMode != MODE_HEAT_VISION) {
          previousMode = currentMode;
          currentMode = MODE_HEAT_VISION;
          lastActivityTime = currentMillis;
          
          #ifdef BUZZER_PIN
          // Sound: Heat Vision Activate (High Pitch Pulse)
          playTone(880, 50, VOL_MODE_CHANGE);
          #endif
      }
  } else { // Released
      if (currentMode == MODE_HEAT_VISION) {
          currentMode = previousMode;
          lastActivityTime = currentMillis;
          
          #ifdef BUZZER_PIN
          // Sound: Heat Vision Deactivate
          playTone(440, 50, VOL_MODE_CHANGE);
          #endif
      }
  }


  // --- Heat Vision Handling ---
  if (currentMode == MODE_HEAT_VISION) {
      runHeatVision();
  }

  // --- Sensor Handling ---
  // Always run sensor to keep measurementData fresh for Fusion
  if (sensor.isDataReady()) {
    if (sensor.getRangingData(&measurementData)) {
      
      // Only do Haptic Logic if NOT in Heat Vision (Heat Vision handles its own haptics)
      if (currentMode != MODE_HEAT_VISION) {
          if (currentMode == MODE_PRECISION) {
        // MODE: Precision Pointer (Check CENTER zones only)
        int targetDistance = 9999;
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

        handleHapticsPrecision(targetDistance);
      } 
      else {
        // MODE: Smart Terrain (Gradient Analysis)
        // Uses 4x4 Matrix to detect patterns (Stairs, Drop-offs, Walls)
        
        // 1. Analyze Rows (Row 0 = Top, Row 3 = Bottom)
        long rowSum[4] = {0, 0, 0, 0};
        int rowCount[4] = {0, 0, 0, 0};
        int rowAvg[4] = {9999, 9999, 9999, 9999};

        for (int i = 0; i < 16; i++) {
            int row = i / 4; // Map 0-15 to Rows 0-3
            // Check for valid status (5=Valid, 9=Valid with low confidence)
            if (measurementData.target_status[i] == 5 || measurementData.target_status[i] == 9) {
                if (measurementData.distance_mm[i] > 0 && measurementData.distance_mm[i] < 4000) {
                    rowSum[row] += measurementData.distance_mm[i];
                    rowCount[row]++;
                }
            }
        }

        // Calculate Averages
        for (int r=0; r<4; r++) {
            if (rowCount[r] > 0) rowAvg[r] = rowSum[r] / rowCount[r];
        }

        int hapticMode = HAPTIC_NONE;
        int intensity = 0; // 0-255

        // --- SCENARIO 1: Looking Forward (Wall Detection) ---
        if (pitch > -15) { 
            // Angle is flat (>-15 deg). We are looking for walls/obstacles ahead.
            // Focus on Top Half (Row 0 & 1) to avoid ground reflections
            int forwardDist = min(rowAvg[0], rowAvg[1]);
            
            if (forwardDist < 1500) {
                hapticMode = HAPTIC_WALL;
                // Map intensity: 1.5m -> 50, 0.3m -> 255
                intensity = map(forwardDist, 300, 1500, 255, 50);
                intensity = constrain(intensity, 0, 255);
            }
        }
        // --- SCENARIO 2: Looking Down (Terrain Analysis) ---
        else {
            // We are looking at the ground. Analyze the Gradient.
            
            // A) Drop-off / Stairs Down Detection
            // Logic: If the bottom row sees ground, but the row above sees "nothing" (far away), it's a cliff.
            // Or: If the measured distance is significantly larger than expected by Trig.
            
            bool isDropOff = false;
            
            // Gradient Check: Row 2 is much further than Row 3 (> 50cm)
            if (rowAvg[3] < 2000 && (rowAvg[2] > rowAvg[3] + 500)) {
                 isDropOff = true;
            }
            
            // Trig Check (Backup): Whole ground is missing
            float angleRad = abs(pitch) * PI / 180.0;
            float expectedDist = HAND_HEIGHT_MM / sin(angleRad);
            if (expectedDist < 3000 && rowAvg[3] > expectedDist + 400) {
                 isDropOff = true;
            }

            if (isDropOff) {
                hapticMode = HAPTIC_DROPOFF;
                intensity = 255; 
            } 
            else {
                // B) Stairs Up / Obstacle Detection
                // Logic: Check for "Stair Step" pattern: Row 3 < Row 2 < Row 1
                // The distance increases as we look up, but in discrete steps.
                
                int step1 = rowAvg[2] - rowAvg[3]; // Height of first step (approx distance diff)
                int step2 = rowAvg[1] - rowAvg[2]; // Height of second step
                
                // Note: Distance diff is not exactly height, but proportional. 
                // 100mm - 300mm diff usually indicates a step when looking down at ~45 deg.
                
                if (step1 > STAIR_STEP_MIN_HEIGHT && step1 < STAIR_STEP_MAX_HEIGHT &&
                    step2 > STAIR_STEP_MIN_HEIGHT && step2 < STAIR_STEP_MAX_HEIGHT) {
                    // It's a staircase! (Multiple steps)
                    hapticMode = HAPTIC_STAIRS_UP;
                    intensity = 200;
                }
                else if (step1 > STAIR_STEP_MIN_HEIGHT) {
                    // Just one step or obstacle (Curb)
                    hapticMode = HAPTIC_OBSTACLE;
                    intensity = 200;
                }
                // C) Flat Ground
                else {
                    hapticMode = HAPTIC_NONE;
                }
            }
        }

        // --- BAT-IN-DANGER MODE (Glass / Mirror Detection) ---
        // Like a bat confused by a window, the sensor gets weird reflections.
        // Check center zones (5,6,9,10) for specific error codes
        bool glassDetected = false;
        int centerZones[] = {5, 6, 9, 10};
        for (int i : centerZones) {
            // Status 4 (Phase Fail) or 13 (Consistency Fail) often happen with glass/mirrors
            if (measurementData.target_status[i] == 4 || measurementData.target_status[i] == 13) {
                glassDetected = true;
                break;
            }
        }
        
        if (glassDetected && hapticMode == HAPTIC_NONE) {
            // Only warn if we don't have a more urgent alarm (like Dropoff)
            hapticMode = HAPTIC_GLASS;
            // Optional: Short audio ping for glass
            #ifdef BUZZER_PIN
            playTone(2000, 50, VOL_ALARM); 
            #endif
        }

        // --- NEW: GAP HUNTER (Door Finder) ---
        // Only active if looking forward (Pitch > -20) and no other alarm
        if (pitch > -20 && hapticMode == HAPTIC_NONE) {
            
            // Average Left Column (0,4,8,12) vs Right Column (3,7,11,15) vs Center (5,6,9,10)
            long distLeft = 0, distRight = 0, distCenter = 0;
            int countLeft = 0, countRight = 0, countCenter = 0;

            // Helper to sum up valid zones
            for(int r=0; r<4; r++) {
                // Left Col (0, 4, 8, 12)
                int idxL = r*4; 
                if(measurementData.target_status[idxL] == 5) { distLeft += measurementData.distance_mm[idxL]; countLeft++; }
                
                // Right Col (3, 7, 11, 15)
                int idxR = r*4 + 3;
                if(measurementData.target_status[idxR] == 5) { distRight += measurementData.distance_mm[idxR]; countRight++; }
            }
            
            // Center
            for(int i : centerZones) {
                if(measurementData.target_status[i] == 5) { distCenter += measurementData.distance_mm[i]; countCenter++; }
            }

            if (countLeft > 0 && countRight > 0 && countCenter > 0) {
                distLeft /= countLeft;
                distRight /= countRight;
                distCenter /= countCenter;

                // Logic: Left & Right are CLOSE (Frame), Center is FAR (Gap)
                if (distLeft < 1500 && distRight < 1500) { // Frame within 1.5m
                    if (distCenter > (distLeft + GAP_DEPTH_DIFF_MM) && distCenter > (distRight + GAP_DEPTH_DIFF_MM)) {
                        // Found a gap!
                        hapticMode = HAPTIC_GAP;
                    }
                }
            }
        }

        // --- EXECUTE HAPTIC FEEDBACK ---
        
        unsigned long now = millis();

        #ifdef ENABLE_DRV2605
        // DRV2605L Logic (Effects)
        static int lastHapticMode = HAPTIC_NONE;
        static unsigned long lastEffectTrigger = 0;
        
        if (hapticMode == HAPTIC_NONE) {
            // Stop
            if (lastHapticMode != HAPTIC_NONE) {
                drv.stop();
            }
        }
        else if (hapticMode == HAPTIC_WALL) {
            // Continuous Wall -> Use "Buzz" effect repeatedly or RTP
            // Effect 47: Buzz 1 100%
            if (now - lastEffectTrigger > 150) { // Re-trigger every 150ms
                drv.setWaveform(0, 47); 
                drv.setWaveform(1, 0);
                drv.go();
                lastEffectTrigger = now;
            }
        }
        else if (hapticMode == HAPTIC_DROPOFF) {
            // Dropoff -> Transition Ramp Down (Effect 74) or Triple Click (Effect 12)
            // Let's use Ramp Down Long Smooth 1 (74) - feels like falling
            if (now - lastEffectTrigger > 600) { 
                drv.setWaveform(0, 74); 
                drv.setWaveform(1, 0);
                drv.go();
                lastEffectTrigger = now;
            }
        }
        else if (hapticMode == HAPTIC_OBSTACLE) {
            // Obstacle -> Bump (Effect 7) or Double Click (Effect 10)
            // Let's use Soft Bump 100% (15)
            if (now - lastEffectTrigger > 400) { 
                drv.setWaveform(0, 15); 
                drv.setWaveform(1, 0);
                drv.go();
                lastEffectTrigger = now;
            }
        }
        else if (hapticMode == HAPTIC_STAIRS_UP) {
            // Stairs Up -> Ascending Pulse
            // Effect 58: Transition Ramp Up Long Smooth 1
            if (now - lastEffectTrigger > 500) { 
                drv.setWaveform(0, 58); 
                drv.setWaveform(1, 0);
                drv.go();
                lastEffectTrigger = now;
            }
        }
        else if (hapticMode == HAPTIC_GAP) {
            // Gap Found -> Double Click
            // Effect 11: Double Click 100%
            if (now - lastEffectTrigger > 800) { // Slower repeat to not be annoying
                drv.setWaveform(0, 11); 
                drv.setWaveform(1, 0);
                drv.go();
                lastEffectTrigger = now;
            }
        }
        else if (hapticMode == HAPTIC_GLASS) {
            // Glass Warning -> Sharp Tick
            // Effect 1: Strong Click 100%
            if (now - lastEffectTrigger > 300) { 
                drv.setWaveform(0, 1); 
                drv.setWaveform(1, 0);
                drv.go();
                lastEffectTrigger = now;
            }
        }
        lastHapticMode = hapticMode;
        
        #else
        // Standard GPIO Logic (PWM/Strobe)
        if (hapticMode == HAPTIC_NONE) {
            nrf_gpio_pin_write(MOTOR_PIN, 0);
        }
        else if (hapticMode == HAPTIC_WALL) {
            if (intensity > 200) {
                nrf_gpio_pin_write(MOTOR_PIN, 1); // Full ON
            } else {
                // Soft vibration (50% duty cycle at high freq)
                if ((now / 10) % 2 == 0) nrf_gpio_pin_write(MOTOR_PIN, 1);
                else nrf_gpio_pin_write(MOTOR_PIN, 0);
            }
        }
        else if (hapticMode == HAPTIC_DROPOFF) {
            // Fast Strobe (Nervous) -> 50ms ON, 50ms OFF
            if ((now / 50) % 2 == 0) nrf_gpio_pin_write(MOTOR_PIN, 1);
            else nrf_gpio_pin_write(MOTOR_PIN, 0);
        }
        else if (hapticMode == HAPTIC_OBSTACLE) {
            // Slow Heavy Pulse -> 200ms ON, 200ms OFF
            if ((now / 200) % 2 == 0) nrf_gpio_pin_write(MOTOR_PIN, 1);
            else nrf_gpio_pin_write(MOTOR_PIN, 0);
        }
        else if (hapticMode == HAPTIC_STAIRS_UP) {
            // Double Pulse -> 100ms ON, 100ms OFF, 100ms ON, 300ms OFF
            int cycle = now % 600;
            if (cycle < 100 || (cycle > 200 && cycle < 300)) nrf_gpio_pin_write(MOTOR_PIN, 1);
            else nrf_gpio_pin_write(MOTOR_PIN, 0);
        }
        #endif
      }
      
      // Note: Activity Check is now handled by IMU (Gyro)
    }
    } // End if (currentMode != MODE_HEAT_VISION)
  }
}

// enterStandby removed - replaced by goToSleep

void goToSleep() {
    Serial.println("Going to System OFF (Deep Sleep)...");
    delay(100); // Allow serial to flush
    
    #ifdef BUZZER_PIN
    playShutdownMelody();
    #endif
    
    // Configure Wakeup Pins (Sense LOW)
    nrf_gpio_cfg_sense_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(TRIGGER_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    
    // Turn off sensors
    sensor.stopRanging();
    
    // Enter System OFF
    NRF_POWER->SYSTEMOFF = 1;
    while(1); // Wait for shutdown
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
    if (currentMode == MODE_SMART_TERRAIN) {
        currentMode = MODE_PRECISION;
        
        #ifdef BUZZER_PIN
        // Sound: Zoom In (Ascending C5 -> E5)
        playTone(523, 100, VOL_MODE_CHANGE); delay(20);
        playTone(659, 100, VOL_MODE_CHANGE); delay(20);
        #endif

        // Feedback: Double Buzz
        #ifdef ENABLE_DRV2605
        drv.setWaveform(0, 10); // Effect 10: Double Click 100%
        drv.setWaveform(1, 0);  // End
        drv.go();
        #else
        nrf_gpio_pin_write(MOTOR_PIN, 1); delay(100);
        nrf_gpio_pin_write(MOTOR_PIN, 0); delay(100);
        nrf_gpio_pin_write(MOTOR_PIN, 1); delay(100);
        nrf_gpio_pin_write(MOTOR_PIN, 0);
        #endif
    } 
    else {
        currentMode = MODE_SMART_TERRAIN;
        
        #ifdef BUZZER_PIN
        // Sound: Zoom Out (Descending E5 -> C5)
        playTone(659, 100, VOL_MODE_CHANGE); delay(20);
        playTone(523, 100, VOL_MODE_CHANGE); delay(20);
        #endif

        // Feedback: Single Long Buzz
        #ifdef ENABLE_DRV2605
        drv.setWaveform(0, 15); // Effect 15: Soft Bump 100%
        drv.setWaveform(1, 0);  // End
        drv.go();
        #else
        nrf_gpio_pin_write(MOTOR_PIN, 1); delay(300);
        nrf_gpio_pin_write(MOTOR_PIN, 0);
        #endif
    }
    
    Serial.print("Mode Changed: "); Serial.println(currentMode);
}

// handleHapticsNavigation removed - replaced by Smart Terrain logic


void handleHapticsPrecision(int distance) {
    // Analog-like feedback using PWM (simulated via fast switching or real PWM if configured)
    // For simplicity with GPIO HAL, we'll use a "Geiger Counter" style clicking
    // The closer, the faster the clicks.
    
    unsigned long currentMillis = millis();
    int clickInterval = 0;

    if (distance > 2000) {
        // Silence
        #ifndef ENABLE_DRV2605
        nrf_gpio_pin_write(MOTOR_PIN, 0);
        #endif
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
        #ifdef ENABLE_DRV2605
        drv.setWaveform(0, 1); // Effect 1: Strong Click 100%
        drv.setWaveform(1, 0);
        drv.go();
        #else
        nrf_gpio_pin_write(MOTOR_PIN, 1);
        delay(5); 
        nrf_gpio_pin_write(MOTOR_PIN, 0);
        #endif
    }
}

#ifdef BUZZER_PIN
// Helper for Volume Control (Software PWM)
// Volume: 1-10 (1 = Quiet, 10 = Max/50% Duty)
void playTone(unsigned int frequency, unsigned long duration, int volume) {
    if (frequency == 0 || volume == 0) {
        delay(duration);
        return;
    }
    
    unsigned long period = 1000000 / frequency;
    unsigned long onTime = period * volume * 5 / 100; // volume(1-10) * 5%
    unsigned long offTime = period - onTime;
    
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
        nrf_gpio_pin_write(BUZZER_PIN, 1);
        delayMicroseconds(onTime);
        nrf_gpio_pin_write(BUZZER_PIN, 0);
        delayMicroseconds(offTime);
    }
}

void playStartupMelody() {
    // La Marseillaise Opening (Extended)
    // D4, D4, D4, G4, G4, A4, A4, D5 ... B4, B4
    // "Al-lons en-fants de la Pa-trie... Le jour..."
    
    int melody[] = { 294, 294, 294, 392, 392, 440, 440, 587, 494, 494 };
    int durations[] = { 150, 150, 150, 400, 400, 400, 400, 800, 300, 300 };
    
    for (int i = 0; i < 10; i++) {
        playTone(melody[i], durations[i], VOL_STARTUP);
        delay(durations[i] * 0.30); // Gap
    }
}

void playShutdownMelody() {
    // Windows XP Shutdown Style
    // Eb5, Bb4, G4, Eb4
    
    int melody[] = { 622, 466, 392, 311 };
    int durations[] = { 300, 300, 300, 1000 };
    
    for (int i = 0; i < 4; i++) {
        playTone(melody[i], durations[i], VOL_SHUTDOWN);
        delay(durations[i] * 0.10); // Gap
    }
}

void announceBatteryLevel() {
    // Read analog value (0-1023). Reference is 3.3V (internal).
    // We assume a voltage divider is used if battery > 3.3V (e.g. LiPo 4.2V).
    // Common divider: 100k / 100k -> Factor 2.
    // Voltage = (Analog / 1023.0) * 3.3 * 2.0;
    
    int raw = analogRead(BATTERY_PIN);
    float voltage = (raw / 1023.0) * 3.3 * 2.0; 
    
    // Debug
    // Serial.print("Bat Raw: "); Serial.print(raw);
    // Serial.print(" V: "); Serial.println(voltage);

    if (voltage > 4.0) {
        // Full: 4 Beeps
        for(int i=0; i<4; i++) { playTone(1000, 100, VOL_BATTERY); delay(100); }
    } else if (voltage > 3.7) {
        // Good: 3 Beeps
        for(int i=0; i<3; i++) { playTone(1000, 100, VOL_BATTERY); delay(100); }
    } else if (voltage > 3.4) {
        // Low: 2 Beeps
        for(int i=0; i<2; i++) { playTone(1000, 100, VOL_BATTERY); delay(100); }
    } else {
        // Critical: 1 Long Low Beep
        playTone(500, 500, VOL_BATTERY);
    }
}

void playCalibrationSuccess() {
    // Positive Sequence: C5, E5, G5, C6 (Major Arpeggio)
    int melody[] = { 523, 659, 784, 1047 };
    int durations[] = { 100, 100, 100, 300 };
    
    for (int i = 0; i < 4; i++) {
        playTone(melody[i], durations[i], VOL_STARTUP);
        delay(50);
    }
}
#endif

void calibrateIMU() {
    Serial.println("Calibrating IMU...");
    
    // 1. Read current raw pitch (without offset)
    int16_t accelGyro[6]={0};
    float rawPitch = 0;
    
    // Take average of 10 readings
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
        rawPitch = sumPitch / samples;
        
        // 2. Calculate Offset
        // We want Result = Raw + Offset = 0
        // So Offset = -Raw
        pitchOffset = -rawPitch;
        
        Serial.print("New Offset: "); Serial.println(pitchOffset);
        
        // 3. Save to File
        InternalFS.remove("/calib.txt");
        File file = InternalFS.open("/calib.txt", FILE_O_WRITE);
        if (file) {
            file.print(pitchOffset);
            file.close();
            Serial.println("Calibration Saved.");
            
            #ifdef BUZZER_PIN
            playCalibrationSuccess();
            #endif
        } else {
            Serial.println("Failed to save calibration.");
        }
    }
}

void checkForDrop(int16_t* accelGyro) {
    // Calculate G-Force Magnitude
    // Default BMI160 range is usually +/- 2g or 4g. 
    // Assuming +/- 4g range (8192 LSB/g) for safety, or we check raw values.
    // Let's use a raw threshold. 2.5g * 16384 (if 2g range) = 40960 (overflows int16)
    // If range is 2g, we can't detect 2.5g properly without changing range.
    // However, a fall impact usually spikes max values.
    
    long x = accelGyro[3];
    long y = accelGyro[4];
    long z = accelGyro[5];
    
    // Magnitude Squared
    long magSq = x*x + y*y + z*z;
    
    // Threshold: 2.5g. 
    // If 1g = 16384, 1g^2 = 268,435,456.
    // 2.5g^2 = 6.25 * 1g^2 = 1,677,721,600. Fits in long (2B is too small, need 4B).
    // Arduino long is 32-bit (up to 2B). Wait, 2,147,483,647. So it fits.
    
    // Let's use a simplified threshold based on observation or config.
    // If we assume 1g ~ 16000.
    // Impact > 2.5g is huge.
    
    // Let's just check if magnitude is very high.
    // 2.5g * 16384 = 40960.
    // Squared = 1.6e9.
    
    if (magSq > 1600000000) { // Approx 2.5g threshold squared
        if (!dropDetected) {
            dropDetected = true;
            dropTime = millis();
            Serial.println("DROP DETECTED!");
        }
    }
    
    if (dropDetected) {
        unsigned long timeSinceDrop = millis() - dropTime;
        
        if (timeSinceDrop > DROP_WAIT_TIME_MS && timeSinceDrop < (DROP_WAIT_TIME_MS + DROP_ALARM_DURATION_MS)) {
            // Alarm Phase
            if (!dropAlarmActive) {
                dropAlarmActive = true;
                Serial.println("DROP ALARM ACTIVE");
            }
            
            // Play Alarm Sound / Haptics
            #ifdef BUZZER_PIN
            if (millis() % 1000 < 500) {
                playTone(2000, 100, VOL_ALARM);
            }
            #endif
            
            #ifdef ENABLE_DRV2605
            if (millis() % 1000 < 200) {
                drv.setWaveform(0, 47); // Buzz
                drv.setWaveform(1, 0);
                drv.go();
            }
            #endif
        } else if (timeSinceDrop > (DROP_WAIT_TIME_MS + DROP_ALARM_DURATION_MS)) {
            // Timeout - Stop Alarm
            dropDetected = false;
            dropAlarmActive = false;
        }
        
        // Reset if picked up (Orientation changes to upright?)
        // Or just button press resets it.
        // For now, let's say button press resets it (handled in loop).
    }
}

void runHeatVision() {
    // 1. Analyze ToF Width (Object Size)
    // Check the middle horizontal slice (Zones 4,5,6,7)
    int objectWidth = 0;
    int validZones = 0;
    int sumDist = 0;
    
    // We check zones 4,5,6,7 (Row 1)
    for (int i=4; i<=7; i++) {
        if (measurementData.target_status[i] == 5 || measurementData.target_status[i] == 9) {
            if (measurementData.distance_mm[i] < 2000) { // Only consider objects within 2m
                objectWidth++;
                sumDist += measurementData.distance_mm[i];
                validZones++;
            }
        }
    }
    
    int avgDist = (validZones > 0) ? (sumDist / validZones) : 2000;

    // 2. Read pixels
    amg.readPixels(amgPixels);
    
    int hotPixelCount = 0;
    float maxTemp = 0;
    
    for (int i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
        if (amgPixels[i] > HEAT_THRESHOLD_C) {
            hotPixelCount++;
            if (amgPixels[i] > maxTemp) maxTemp = amgPixels[i];
        }
    }
    
    // 3. Classification Logic
    int classification = 0; // 0=None, 1=Small(Cat/Cup), 2=Human, 3=Monitor/Machine
    
    if (hotPixelCount > 0) {
        if (objectWidth >= 3) {
            // Wide object (> 60cm) + Heat -> Monitor / PC Tower / Radiator
            classification = 3; 
        } else if (objectWidth >= 1) {
            // Narrower object (20-40cm) + Heat -> Human Head / Torso
            // Check pixel count to distinguish from small cup
            if (hotPixelCount >= 3) classification = 2; // Human
            else classification = 1; // Small object
        } else {
            // Heat but no ToF object? (Maybe out of range or reflection)
            // Fallback to pixel count
            if (hotPixelCount >= 4) classification = 2;
            else classification = 1;
        }
    }
    
    // 4. Feedback
    unsigned long now = millis();
    
    if (classification == 2) { // HUMAN
        // Feedback: Slow "Heartbeat" (Heavy Pulse)
        // Interval: 1000ms
        if (now % 1000 < 150) {
            #ifdef ENABLE_DRV2605
            drv.setWaveform(0, 12); // Effect 12: Triple Click (Heavy feel)
            drv.setWaveform(1, 0);
            drv.go();
            #else
            analogWrite(MOTOR_PIN, 255);
            #endif
        } else {
            #ifndef ENABLE_DRV2605
            analogWrite(MOTOR_PIN, 0);
            #endif
        }
        Serial.print("Human Detected! Dist: "); Serial.print(avgDist); Serial.print(" Pixels: "); Serial.println(hotPixelCount);
    } 
    else if (classification == 3) { // MONITOR / MACHINE
        // Feedback: Mechanical "Double Tick" (Artificial feel)
        // Interval: 1000ms
        if (now % 1000 < 100) {
            #ifdef ENABLE_DRV2605
            drv.setWaveform(0, 52); // Pulsing Strong 1
            drv.setWaveform(1, 0);
            drv.go();
            #else
            analogWrite(MOTOR_PIN, 100);
            #endif
        } else {
            #ifndef ENABLE_DRV2605
            analogWrite(MOTOR_PIN, 0);
            #endif
        }
        Serial.print("Monitor Detected. Dist: "); Serial.print(avgDist); Serial.print(" Width: "); Serial.println(objectWidth);
    }
    else if (classification == 1) { // SMALL OBJECT
        // Feedback: Fast "Geiger Counter" Ticking
        // Interval: 200ms
        if (now % 200 < 50) {
            #ifdef ENABLE_DRV2605
            drv.setWaveform(0, 1); // Effect 1: Strong Click (Sharp feel)
            drv.setWaveform(1, 0);
            drv.go();
            #else
            analogWrite(MOTOR_PIN, 150);
            #endif
        } else {
            #ifndef ENABLE_DRV2605
            analogWrite(MOTOR_PIN, 0);
            #endif
        }
        Serial.print("Small Object. Dist: "); Serial.print(avgDist); Serial.print(" Pixels: "); Serial.println(hotPixelCount);
    } else {
        #ifndef ENABLE_DRV2605
        analogWrite(MOTOR_PIN, 0);
        #endif
    }
}
