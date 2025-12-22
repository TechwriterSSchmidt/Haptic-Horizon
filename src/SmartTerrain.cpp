#include "SmartTerrain.h"
#include <math.h>

// Define XSHUT pins for STM32duino library (Not used in Dual-Bus mode, but required by constructor)
// We pass -1 to indicate "Not Used" / "Software Controlled"
#define XSHUT_PIN_MATRIX -1 
#define XSHUT_PIN_FOCUS -1

SmartTerrain::SmartTerrain() {
    _sensorMatrix = nullptr;
    _sensorFocus = nullptr;
    _hapticMode = HAPTIC_NONE;
    _hapticInterval = 150;
    _wallDirection = 0;
    _thermalClass = THERMAL_NONE;
}

bool SmartTerrain::begin(TwoWire *wirePrimary, TwoWire *wireSecondary) {
    bool success = true;
    
    // --- 1. Init Focus Sensor (VL53L4CX) on Primary Bus (Wire) ---
    if (_sensorFocus == nullptr) {
        _sensorFocus = new VL53L4CX(wirePrimary, XSHUT_PIN_FOCUS);
    }
    
    if (_sensorFocus->begin() != 0) { // 0 = Success in STM32duino lib
        Serial.println("SmartTerrain: VL53L4CX (Focus) not found on Bus 1");
        success = false;
    } else {
        _sensorFocus->VL53L4CX_Off(); // Reset
        _sensorFocus->VL53L4CX_On();
        _sensorFocus->VL53L4CX_StartMeasurement();
        Serial.println("SmartTerrain: Focus Sensor OK");
    }

    // --- 2. Init Matrix Sensor (VL53L8CX) on Secondary Bus (Wire1) ---
    if (_sensorMatrix == nullptr) {
        _sensorMatrix = new VL53L8CX(wireSecondary, XSHUT_PIN_MATRIX);
    }
    
    if (_sensorMatrix->begin() != 0) { // 0 = Success
        Serial.println("SmartTerrain: VL53L8CX (Matrix) not found on Bus 2");
        success = false;
    } else {
        _sensorMatrix->init_sensor();
        _sensorMatrix->vl53l8cx_start_ranging();
        Serial.println("SmartTerrain: Matrix Sensor OK");
    }

    // --- 3. Init Thermal (MLX90640) on Secondary Bus (Wire1) ---
    if (!_mlx.begin(MLX90640_I2CADDR_DEFAULT, wireSecondary)) {
        Serial.println("SmartTerrain: MLX90640 not found on Bus 2");
        success = false;
    } else {
        _mlx.setMode(MLX90640_CHESS);
        _mlx.setResolution(MLX90640_ADC_18BIT);
        _mlx.setRefreshRate(MLX90640_2_HZ);
        Serial.println("SmartTerrain: Thermal Cam OK");
    }
    
    return success;
}

void SmartTerrain::stop() {
    if (_sensorMatrix) _sensorMatrix->vl53l8cx_stop_ranging();
    if (_sensorFocus) _sensorFocus->VL53L4CX_StopMeasurement();
}

void SmartTerrain::start() {
    if (_sensorMatrix) _sensorMatrix->vl53l8cx_start_ranging();
    if (_sensorFocus) _sensorFocus->VL53L4CX_StartMeasurement();
}

void SmartTerrain::updateThermal() {
    // 1. Analyze ToF Width (Object Size) - Sensor Fusion
    // Using Matrix Sensor Data (8x8)
    int objectWidth = 0;
    int validZones = 0;
    int sumDist = 0;
    
    // Check center row (Zones 24-31 in 8x8, or simplified 4x4 logic)
    // VL53L8CX 8x8 mode: 64 zones. Center rows are 3 and 4.
    // Indices: Row 3 (24-31), Row 4 (32-39)
    
    for (int i=28; i<=35; i++) { // Center horizontal strip
        if (_matrixData.target_status[i] == 5 || _matrixData.target_status[i] == 9) {
            if (_matrixData.distance_mm[i] < 2000) {
                objectWidth++;
                sumDist += _matrixData.distance_mm[i];
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
    if (!_sensorMatrix || !_sensorFocus) return;

    // Check Data Ready
    uint8_t readyMatrix = 0;
    uint8_t readyFocus = 0;
    
    _sensorMatrix->vl53l8cx_check_data_ready(&readyMatrix);
    _sensorFocus->VL53L4CX_GetMeasurementDataReady(&readyFocus);
    
    if (!readyMatrix && !readyFocus) return;

    // Get Data
    if (readyMatrix) _sensorMatrix->vl53l8cx_get_ranging_data(&_matrixData);
    if (readyFocus) {
        _sensorFocus->VL53L4CX_GetMultiRangingData(&_focusData);
        _sensorFocus->VL53L4CX_ClearInterruptAndStartMeasurement();
    }

    // --- FUSION LOGIC ---
    
    // 1. Focus Sensor (The "Pointer")
    int focusDist = 9999;
    if (_focusData.NumberOfObjectsFound > 0) {
        if (_focusData.RangeData[0].RangeStatus == 0) {
            focusDist = _focusData.RangeData[0].RangeMilliMeter;
        }
    }

    // 2. Matrix Sensor (The "Scanner")
    // Calculate average of center zones for general obstacle detection
    long matrixSum = 0;
    int matrixCount = 0;
    // Use center 4x4 block of the 8x8 matrix
    // Rows 2-5, Cols 2-5
    for (int r=2; r<=5; r++) {
        for (int c=2; c<=5; c++) {
            int idx = r*8 + c;
            if (_matrixData.target_status[idx] == 5) {
                matrixSum += _matrixData.distance_mm[idx];
                matrixCount++;
            }
        }
    }
    int matrixDist = (matrixCount > 0) ? (matrixSum / matrixCount) : 9999;

    _hapticMode = HAPTIC_NONE;
    _wallDirection = 0;

    // --- HAPTIC DECISION TREE ---

    // PRIORITY 1: Focus Sensor (Precise Object)
    // If Focus sees something CLOSE (< 1.5m) and closer than Matrix average
    if (focusDist < 1500 && focusDist < matrixDist - 200) {
        // "I am pointing at something specific"
        _hapticMode = HAPTIC_GLASS; // Sharp Tick for precise objects
        _hapticInterval = map(focusDist, 100, 1500, 50, 500);
    }
    // PRIORITY 2: Matrix Sensor (General Wall/Obstacle)
    else if (matrixDist < 2000) {
        _hapticMode = HAPTIC_WALL;
        _hapticInterval = map(matrixDist, 300, 2000, 50, 400);
        
        // Direction Logic (Left vs Right half of Matrix)
        long sumLeft = 0, sumRight = 0;
        int countLeft = 0, countRight = 0;
        for (int i=0; i<32; i++) { // Left Half (Cols 0-3)
             if ((i%8) < 4 && _matrixData.target_status[i] == 5) { sumLeft += _matrixData.distance_mm[i]; countLeft++; }
        }
        for (int i=0; i<32; i++) { // Right Half (Cols 4-7)
             if ((i%8) >= 4 && _matrixData.target_status[i] == 5) { sumRight += _matrixData.distance_mm[i]; countRight++; }
        }
        
        if (countLeft > 0 && countRight > 0) {
            int avgLeft = sumLeft / countLeft;
            int avgRight = sumRight / countRight;
            if (avgLeft < avgRight - 300) _wallDirection = -1;
            else if (avgRight < avgLeft - 300) _wallDirection = 1;
        }
    }
    
    // PRIORITY 3: Drop-Off Detection (using Matrix bottom rows)
    // If pitch is looking down, check for "missing ground"
    if (pitch < -20) {
        // Check bottom row (Indices 56-63)
        int groundCount = 0;
        for (int i=56; i<64; i++) {
            if (_matrixData.target_status[i] == 5 && _matrixData.distance_mm[i] < 2000) groundCount++;
        }
        if (groundCount == 0) {
            _hapticMode = HAPTIC_DROPOFF; // ALARM! Ground lost!
        }
    }
}

bool SmartTerrain::isDataReady() {
    return true; // Handled internally
}

int SmartTerrain::getHapticMode() { return _hapticMode; }
int SmartTerrain::getHapticInterval() { return _hapticInterval; }
int SmartTerrain::getWallDirection() { return _wallDirection; }
int SmartTerrain::getThermalClass() { return _thermalClass; }
int SmartTerrain::getThermalCenter() { return _thermalCenter; }
int SmartTerrain::getThermalDist() { return _thermalDist; }

