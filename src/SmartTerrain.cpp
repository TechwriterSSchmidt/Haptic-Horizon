#include "SmartTerrain.h"
#include <math.h>

SmartTerrain::SmartTerrain() {
    _hapticMode = HAPTIC_NONE;
    _hapticInterval = 150;
    _wallDirection = 0;
    _thermalClass = THERMAL_NONE;
}

bool SmartTerrain::begin(TwoWire *wirePrimary, TwoWire *wireSecondary) {
    bool success = true;
    
    // Init ToF
    // SparkFun VL53L5CX begin() uses the Wire instance passed to it, or default Wire if none.
    // We need to make sure we pass the correct wire if the library supports it in begin().
    // Looking at the library header (implied), begin() usually takes address and wire.
    // If not, we might need setWire. Assuming standard SparkFun pattern:
    if (_sensor.begin() == false) { 
        Serial.println("SmartTerrain: VL53L5CX not found");
        success = false;
    } else {
        _sensor.setResolution(SENSOR_RESOLUTION);
        _sensor.setRangingFrequency(SENSOR_RANGING_FREQ);
        _sensor.startRanging();
    }

    // Init Thermal
    if (!_mlx.begin(MLX90640_I2CADDR_DEFAULT, wireSecondary)) {
        Serial.println("SmartTerrain: MLX90640 not found");
        success = false;
    } else {
        _mlx.setMode(MLX90640_CHESS);
        _mlx.setResolution(MLX90640_ADC_18BIT);
        _mlx.setRefreshRate(MLX90640_2_HZ);
    }
    
    return success;
}

void SmartTerrain::stop() {
    _sensor.stopRanging();
}

void SmartTerrain::start() {
    _sensor.startRanging();
}

void SmartTerrain::updateThermal() {
    // 1. Analyze ToF Width (Object Size) - Sensor Fusion
    int objectWidth = 0;
    int validZones = 0;
    int sumDist = 0;
    
    // Check zones 4,5,6,7 (Row 1)
    for (int i=4; i<=7; i++) {
        if (_measurementData.target_status[i] == 5 || _measurementData.target_status[i] == 9) {
            if (_measurementData.distance_mm[i] < 2000) {
                objectWidth++;
                sumDist += _measurementData.distance_mm[i];
                validZones++;
            }
        }
    }
    _thermalDist = (validZones > 0) ? (sumDist / validZones) : 2000;
    _thermalWidth = objectWidth;

    // 2. Read pixels
    // Note: getFrame is blocking on I2C.
    if (_mlx.getFrame(_mlxPixels) != 0) return;
    
    int hotPixelCount = 0;
    float maxTemp = 0;
    long sumX = 0;
    bool dangerHot = false;
    
    for (int i=0; i<768; i++) {
        if (_mlxPixels[i] > HEAT_DANGER_C) dangerHot = true;
        if (_mlxPixels[i] > HEAT_THRESHOLD_C) {
            hotPixelCount++;
            sumX += (i % 32);
            if (_mlxPixels[i] > maxTemp) maxTemp = _mlxPixels[i];
        }
    }
    
    if (hotPixelCount > 0) _thermalCenter = sumX / hotPixelCount;
    else _thermalCenter = 16;
    
    // 3. Classification
    _thermalClass = THERMAL_NONE;
    if (dangerHot) _thermalClass = THERMAL_DANGER;
    else if (hotPixelCount > 0) {
        if (objectWidth >= 3) _thermalClass = THERMAL_MACHINE;
        else if (objectWidth >= 1 && hotPixelCount >= 20) _thermalClass = THERMAL_HUMAN;
        else _thermalClass = THERMAL_SMALL;
    }
}

void SmartTerrain::update(float pitch) {
    if (!_sensor.isDataReady()) return;
    if (!_sensor.getRangingData(&_measurementData)) return;

    // --- Logic from main.cpp moved here ---
    long rowSum[4] = {0};
    int rowCount[4] = {0};
    int rowAvg[4] = {9999, 9999, 9999, 9999};

    for (int i = 0; i < 16; i++) {
        int row = i / 4;
        if (_measurementData.target_status[i] == 5 || _measurementData.target_status[i] == 9) {
            if (_measurementData.distance_mm[i] > 0 && _measurementData.distance_mm[i] < 4000) {
                rowSum[row] += _measurementData.distance_mm[i];
                rowCount[row]++;
            }
        }
    }
    for (int r=0; r<4; r++) if (rowCount[r] > 0) rowAvg[r] = rowSum[r] / rowCount[r];

    _hapticMode = HAPTIC_NONE;
    _wallDirection = 0;

    // SCENARIO 1: Forward
    if (pitch > -15) {
        int forwardDist = min(rowAvg[0], rowAvg[1]);
        if (forwardDist < 1500) {
            _hapticMode = HAPTIC_WALL;
            _hapticInterval = map(forwardDist, 300, 1500, 50, 400);
            _hapticInterval = constrain(_hapticInterval, 50, 400);

            // Direction Logic
            long sumLeft = 0, sumRight = 0;
            int countLeft = 0, countRight = 0;
            if (_measurementData.target_status[0] == 5) { sumLeft += _measurementData.distance_mm[0]; countLeft++; }
            if (_measurementData.target_status[4] == 5) { sumLeft += _measurementData.distance_mm[4]; countLeft++; }
            if (_measurementData.target_status[3] == 5) { sumRight += _measurementData.distance_mm[3]; countRight++; }
            if (_measurementData.target_status[7] == 5) { sumRight += _measurementData.distance_mm[7]; countRight++; }
            
            if (countLeft > 0 && countRight > 0) {
                int avgLeft = sumLeft / countLeft;
                int avgRight = sumRight / countRight;
                if (avgLeft < avgRight - 300) _wallDirection = -1;
                else if (avgRight < avgLeft - 300) _wallDirection = 1;
            }
        }
    } 
    // SCENARIO 2: Down
    else {
        bool isDropOff = false;
        if (rowAvg[3] < 2000 && (rowAvg[2] > rowAvg[3] + 500)) isDropOff = true;
        float angleRad = abs(pitch) * PI / 180.0;
        float expectedDist = HAND_HEIGHT_MM / sin(angleRad);
        if (expectedDist < 3000 && rowAvg[3] > expectedDist + 400) isDropOff = true;

        if (isDropOff) _hapticMode = HAPTIC_DROPOFF;
        else {
            int step1 = rowAvg[2] - rowAvg[3];
            if (step1 > STAIR_STEP_MIN_HEIGHT) _hapticMode = HAPTIC_OBSTACLE;
            if (step1 > STAIR_STEP_MIN_HEIGHT && (rowAvg[1] - rowAvg[2]) > STAIR_STEP_MIN_HEIGHT) _hapticMode = HAPTIC_STAIRS_UP;
        }
    }
    
    // Glass Detection
    bool glassDetected = false;
    int centerZones[] = {5, 6, 9, 10};
    for (int i : centerZones) {
        if (_measurementData.target_status[i] == 4 || _measurementData.target_status[i] == 13) glassDetected = true;
    }
    if (glassDetected && _hapticMode == HAPTIC_NONE) _hapticMode = HAPTIC_GLASS;
}

int SmartTerrain::getHapticMode() { return _hapticMode; }
int SmartTerrain::getHapticInterval() { return _hapticInterval; }
int SmartTerrain::getWallDirection() { return _wallDirection; }
int SmartTerrain::getThermalClass() { return _thermalClass; }
int SmartTerrain::getThermalCenter() { return _thermalCenter; }
int SmartTerrain::getThermalDist() { return _thermalDist; }
bool SmartTerrain::isDataReady() { return _sensor.isDataReady(); }
