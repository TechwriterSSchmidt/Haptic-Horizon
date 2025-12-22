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
    
    // Defaults
    _isOutdoor = false; // Default to Indoor Profile
    _wakeUpTimer = 0;
    _inRestMode = false;
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

void SmartTerrain::toggleProfile() {
    _isOutdoor = !_isOutdoor;
    Serial.print("Profile Switched: ");
    Serial.println(_isOutdoor ? "OUTDOOR" : "INDOOR");
}

bool SmartTerrain::isOutdoorProfile() {
    return _isOutdoor;
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

    // --- STEP 1: IMU STATE MACHINE & LOW-PASS FILTER ---
    // Zone 3: Rest Mode (Vertical Down, < -65 degrees)
    if (pitch < -65.0) {
        _inRestMode = true;
        _wakeUpTimer = 0; // Reset timer
        _hapticMode = HAPTIC_NONE;
        return; // Stop processing
    } 
    else {
        // We are in a potential Active Zone (> -65)
        if (_wakeUpTimer == 0) {
            _wakeUpTimer = millis(); // Start timer
        }
        
        // Check if we have been stable for 600ms
        if (millis() - _wakeUpTimer < 600) {
            // Still waiting (Anti-Pendulum)
            _inRestMode = true;
            _hapticMode = HAPTIC_NONE;
            return;
        }
        
        // Timer expired, we are officially ACTIVE
        _inRestMode = false;
    }

    // --- STEP 2: SENSOR DATA ACQUISITION ---
    // Only read sensors if we are active
    
    uint8_t readyMatrix = 0;
    uint8_t readyFocus = 0;
    
    _sensorMatrix->vl53l8cx_check_data_ready(&readyMatrix);
    _sensorFocus->VL53L4CX_GetMeasurementDataReady(&readyFocus);
    
    if (!readyMatrix && !readyFocus) return;

    if (readyMatrix) _sensorMatrix->vl53l8cx_get_ranging_data(&_matrixData);
    if (readyFocus) {
        _sensorFocus->VL53L4CX_GetMultiRangingData(&_focusData);
        _sensorFocus->VL53L4CX_ClearInterruptAndStartMeasurement();
    }

    // --- STEP 3: DATA PROCESSING ---
    
    // Profile Settings
    int maxRange = _isOutdoor ? 4000 : 2000;
    int warnRange = _isOutdoor ? 2500 : 1200;

    // 1. Focus Sensor Data
    int focusDist = 9999;
    if (_focusData.NumberOfObjectsFound > 0) {
        if (_focusData.RangeData[0].RangeStatus == 0) {
            focusDist = _focusData.RangeData[0].RangeMilliMeter;
        }
    }

    // 2. Ultrasonic Data (MaxBotix / GY-US42 I2C)
    // Address 0x70, Command 0x51 (Take Range Reading)
    long usDist = 9999;
    if (pitch > -20.0) { // Only needed in Scan Mode
        Wire.beginTransmission(0x70);
        Wire.write(0x51);
        Wire.endTransmission();
        // Note: We don't wait here to avoid blocking. We read the PREVIOUS measurement or assume fast response.
        // For better sync, we could add a small delay or state machine, but for now we poll.
        
        Wire.requestFrom(0x70, 2);
        if (Wire.available() >= 2) {
            byte high = Wire.read();
            byte low = Wire.read();
            usDist = ((high << 8) | low) * 10; // Convert cm to mm
        }
    }

    // 2. Matrix Sensor Data (Center Average)
    long matrixSum = 0;
    int matrixCount = 0;
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

    // --- STEP 4: ZONE LOGIC ---

    // ZONE 1: SCAN MODE (Horizontal, > -20 degrees)
    // Priority: Thermal (Human) > Glass (Ultrasonic) > Focus Sensor (Precision)
    if (pitch > -20.0) {
        // A. Thermal Override (Heat Vision)
        if (_thermalClass == THERMAL_HUMAN) {
            _hapticMode = HAPTIC_HUMAN;
            // Direction Logic for Stereo Haptics
            if (_thermalCenter < 10) _wallDirection = -1;
            else if (_thermalCenter > 22) _wallDirection = 1;
            else _wallDirection = 0;
        }
        // B. Glass Detection (Ultrasonic Override)
        // Laser sees far (>2m) BUT Ultrasonic sees near (<1m)
        else if (focusDist > 2000 && usDist > 50 && usDist < 1000) {
            _hapticMode = HAPTIC_GLASS; // Sharp Tick
            _hapticInterval = map(usDist, 50, 1000, 50, 300);
        }
        // C. Focus Sensor (Precision)
        else if (focusDist < maxRange) {
            _hapticMode = HAPTIC_GLASS; // Sharp Tick
            _hapticInterval = map(focusDist, 100, maxRange, 50, 600);
        } 
        else if (matrixDist < warnRange) {
            // Fallback to Matrix if Focus misses but something is there
            _hapticMode = HAPTIC_WALL;
            _hapticInterval = map(matrixDist, 300, warnRange, 100, 500);
        }
    }
    // ZONE 2: WALK MODE (Diagonal, -65 to -20 degrees)
    // Priority: Matrix Sensor (Obstacles) + Drop-Off
    else {
        // A. Obstacle Detection (Matrix)
        if (matrixDist < warnRange) {
            _hapticMode = HAPTIC_WALL;
            _hapticInterval = map(matrixDist, 300, warnRange, 50, 400);
            
            // Direction Logic
            long sumLeft = 0, sumRight = 0;
            int countLeft = 0, countRight = 0;
            for (int i=0; i<32; i++) { 
                 if ((i%8) < 4 && _matrixData.target_status[i] == 5) { sumLeft += _matrixData.distance_mm[i]; countLeft++; }
            }
            for (int i=0; i<32; i++) { 
                 if ((i%8) >= 4 && _matrixData.target_status[i] == 5) { sumRight += _matrixData.distance_mm[i]; countRight++; }
            }
            if (countLeft > 0 && countRight > 0) {
                int avgLeft = sumLeft / countLeft;
                int avgRight = sumRight / countRight;
                if (avgLeft < avgRight - 300) _wallDirection = -1;
                else if (avgRight < avgLeft - 300) _wallDirection = 1;
            }
        }

        // B. Drop-Off Detection (Sensor Fusion)
        // Only active in Walk Mode
        int groundCount = 0;
        for (int i=56; i<64; i++) { // Bottom Row
            if (_matrixData.target_status[i] == 5 && _matrixData.distance_mm[i] < 2500) groundCount++;
        }

        float angleRad = abs(pitch) * PI / 180.0;
        float expectedDist = HAND_HEIGHT_MM / sin(angleRad);
        bool focusLostGround = (focusDist > expectedDist + 450) || (focusDist > 3000);

        if (groundCount == 0 || (focusLostGround && groundCount < 3)) {
            _hapticMode = HAPTIC_DROPOFF; // Override Obstacle Warning
        }
    }
}

bool SmartTerrain::isDataReady() {
    return true; 
}

int SmartTerrain::getHapticMode() { return _hapticMode; }
int SmartTerrain::getHapticInterval() { return _hapticInterval; }
int SmartTerrain::getWallDirection() { return _wallDirection; }
int SmartTerrain::getThermalClass() { return _thermalClass; }
int SmartTerrain::getThermalCenter() { return _thermalCenter; }
int SmartTerrain::getThermalDist() { return _thermalDist; }

