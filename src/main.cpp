#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "config.h"
// Include Nordic GPIO HAL to bypass Arduino pin mapping and use P0.xx directly
#include <hal/nrf_gpio.h>

SparkFun_VL53L5CX sensor;
VL53L5CX_ResultsData measurementData;

unsigned long lastPulseTime = 0;
int pulseState = 0; // 0 = Low, 1 = High

// Forward declaration
void handleHaptics(int distance);

void setup() {
  Serial.begin(115200);
  
  // Motor Setup (using Nordic HAL for direct GPIO control of P0.06)
  nrf_gpio_cfg_output(MOTOR_PIN);
  nrf_gpio_pin_write(MOTOR_PIN, 0);

  // I2C Setup
  // We must specify the pins because we are using a non-standard board definition
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_FREQUENCY); 

  Serial.println("Initializing VL53L5CX...");
  
  // Sensor Setup
  if (sensor.begin() == false) {
    Serial.println("Sensor not found - check wiring. Freezing");
    while (1);
  }

  // Configure Sensor
  sensor.setResolution(SENSOR_RESOLUTION); 
  sensor.setRangingFrequency(SENSOR_RANGING_FREQ);
  sensor.startRanging();

  Serial.println("Haptic Horizon Started (Wide Field of View Mode)");
}

void loop() {
  // Poll for data
  if (sensor.isDataReady()) {
    if (sensor.getRangingData(&measurementData)) {
      
      // Find the closest object in ANY of the 16 zones
      int closestDistance = 9999;
      int validZones = 0;

      for (int i = 0; i < 16; i++) {
        // Status 5 = Target valid, 9 = Target valid with low confidence
        if (measurementData.target_status[i] == 5 || measurementData.target_status[i] == 9) {
          int dist = measurementData.distance_mm[i];
          if (dist < closestDistance && dist > 0) {
            closestDistance = dist;
          }
          validZones++;
        }
      }

      if (validZones > 0) {
        Serial.print("Closest Object: ");
        Serial.print(closestDistance);
        Serial.println(" mm");
        
        handleHaptics(closestDistance);
      } else {
        handleHaptics(9999);
      }
    }
  }
}

void handleHaptics(int distance) {
  unsigned long currentMillis = millis();
  int pulseInterval = 0;

  // Logic for Haptic Feedback
  if (distance > DIST_FAR) {
    // Too far, no vibration
    nrf_gpio_pin_write(MOTOR_PIN, 0);
    pulseState = 0;
    return;
  } 
  else if (distance > DIST_MEDIUM) {
    // Medium range (1m - 2m): Slow Pulse
    pulseInterval = 600; // ms
  } 
  else if (distance > DIST_CLOSE) {
    // Close range (30cm - 1m): Fast Pulse
    pulseInterval = map(distance, DIST_CLOSE, DIST_MEDIUM, 100, 600);
  } 
  else {
    // Critical range (< 30cm): Continuous
    nrf_gpio_pin_write(MOTOR_PIN, 1);
    pulseState = 1;
    return; 
  }

  // Non-blocking Pulse Logic
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
