#include "HapticEngine.h"

HapticEngine::HapticEngine() {
    _lastTrigger = 0;
    _lastMode = 0;
    #ifdef ENABLE_STEREO_HAPTICS
    _stereoEnabled = false;
    #endif
}

bool HapticEngine::begin(TwoWire *wirePrimary, TwoWire *wireSecondary) {
    #ifdef ENABLE_DRV2605
    bool success = true;
    
    // Init Main Driver
    if (!_drvMain.begin(wirePrimary)) {
        Serial.println("Haptic: Main DRV2605L not found!");
        success = false;
    } else {
        _drvMain.selectLibrary(1);
        _drvMain.setMode(DRV2605_MODE_INTTRIG);
        #ifdef DRV2605_MOTOR_TYPE_LRA
        _drvMain.useLRA();
        #else
        _drvMain.useERM();
        #endif
        // Voltage Config
        uint8_t rated = (uint8_t)(MOTOR_RATED_VOLTAGE / 0.02118);
        uint8_t clamp = (uint8_t)(MOTOR_CLAMP_VOLTAGE / 0.02159);
        _drvMain.writeRegister8(DRV2605_REG_RATEDV, rated);
        _drvMain.writeRegister8(DRV2605_REG_CLAMPV, clamp);
        
        // Run Auto-Calibration
        Serial.println("Haptic: Calibrating Main Motor...");
        runAutoCalibration(_drvMain);
    }

    // Init Secondary Driver (Stereo)
    #ifdef ENABLE_STEREO_HAPTICS
    if (wireSecondary != nullptr) {
        if (_drvSec.begin(wireSecondary)) {
            _stereoEnabled = true;
            _drvSec.selectLibrary(1);
            _drvSec.setMode(DRV2605_MODE_INTTRIG);
            #ifdef DRV2605_MOTOR_TYPE_LRA
            _drvSec.useLRA();
            #else
            _drvSec.useERM();
            #endif
            uint8_t rated = (uint8_t)(MOTOR_RATED_VOLTAGE / 0.02118);
            uint8_t clamp = (uint8_t)(MOTOR_CLAMP_VOLTAGE / 0.02159);
            _drvSec.writeRegister8(DRV2605_REG_RATEDV, rated);
            _drvSec.writeRegister8(DRV2605_REG_CLAMPV, clamp);
            
            Serial.println("Haptic: Calibrating Secondary Motor...");
            runAutoCalibration(_drvSec);
            
            Serial.println("Haptic: Stereo Enabled");
        } else {
            Serial.println("Haptic: Secondary DRV2605L not found.");
        }
    }
    #endif

    return success;
    #else
    return true;
    #endif
}

void HapticEngine::stop() {
    #ifdef ENABLE_DRV2605
    _drvMain.stop();
    #ifdef ENABLE_STEREO_HAPTICS
    if (_stereoEnabled) _drvSec.stop();
    #endif
    #endif
}

void HapticEngine::playEffect(HapticEffect effect, int direction) {
    #ifdef ENABLE_DRV2605
    // 0=Center, -1=Left, 1=Right
    
    // Left Motor
    if (direction <= 0) {
        _drvMain.setWaveform(0, (uint8_t)effect);
        _drvMain.setWaveform(1, 0);
        _drvMain.go();
    }
    
    // Right Motor
    #ifdef ENABLE_STEREO_HAPTICS
    if (_stereoEnabled && direction >= 0) {
        _drvSec.setWaveform(0, (uint8_t)effect);
        _drvSec.setWaveform(1, 0);
        _drvSec.go();
    }
    #endif
    #endif
}

void HapticEngine::playPattern(int mode, int intensity, int interval, int direction) {
    unsigned long now = millis();
    
    // Logic for repeating patterns based on mode
    if (mode == HAPTIC_NONE) {
        if (_lastMode != HAPTIC_NONE) stop();
    }
    else if (mode == HAPTIC_WALL) {
        if (now - _lastTrigger > (unsigned long)interval) {
            playEffect(EFFECT_BUZZ, direction);
            _lastTrigger = now;
        }
    }
    else if (mode == HAPTIC_DROPOFF) {
        if (now - _lastTrigger > 600) {
            playEffect(EFFECT_FALLING, direction);
            _lastTrigger = now;
        }
    }
    else if (mode == HAPTIC_OBSTACLE) {
        if (now - _lastTrigger > 400) {
            playEffect(EFFECT_SOFT_BUMP, direction);
            _lastTrigger = now;
        }
    }
    else if (mode == HAPTIC_STAIRS_UP) {
        if (now - _lastTrigger > 500) {
            playEffect(EFFECT_TRANSITION_RAMP, direction);
            _lastTrigger = now;
        }
    }
    else if (mode == HAPTIC_GAP) {
        if (now - _lastTrigger > 800) {
            playEffect(EFFECT_DOUBLE_CLICK, direction);
            _lastTrigger = now;
        }
    }
    else if (mode == HAPTIC_GLASS) {
        if (now - _lastTrigger > 300) {
            playEffect(EFFECT_STRONG_CLICK, direction);
            _lastTrigger = now;
        }
    }
    
    _lastMode = mode;
}

void HapticEngine::playStartup() {
    playEffect(EFFECT_RAMP_UP, 0);
}

void HapticEngine::playShutdown() {
    playEffect(EFFECT_RAMP_DOWN, 0);
    delay(500);
}

void HapticEngine::playCalibrationSuccess() {
    playEffect(EFFECT_TRIPLE_CLICK, 0);
}

void HapticEngine::playFindMe() {
    // Pulsing alarm
    for(int i=0; i<5; i++) {
        playEffect(EFFECT_PULSING_SHARP, 0);
        delay(1000);
    }
}

void HapticEngine::playBatteryLevel(float voltage) {
    // Ensure motors are stopped before this is called (handled in main)
    int pulses = 0;
    if (voltage > 4.0) pulses = 4;      // Full
    else if (voltage > 3.7) pulses = 3; // Good
    else if (voltage > 3.4) pulses = 2; // Low
    else pulses = 1;                    // Critical

    for(int i=0; i<pulses; i++) {
        playEffect(EFFECT_STRONG_CLICK, 0);
        delay(400); 
    }
    
    if (pulses == 1) {
        delay(500);
        playEffect(EFFECT_PULSING_SHARP, 0); // Critical Warning
    }
}

#ifdef ENABLE_DRV2605
void HapticEngine::runAutoCalibration(Adafruit_DRV2605 &drv) {
    // 1. Set Mode to Auto-Calibration
    drv.setMode(DRV2605_MODE_AUTOCAL);

    // 2. Configure Auto-Cal Parameters (Optional, defaults usually work)
    // RATEDV and CLAMPV must be set before this (already done in begin)
    
    // 3. Start Calibration
    drv.go();

    // 4. Wait for completion (GO bit clears)
    // Timeout safety to prevent hanging
    unsigned long start = millis();
    while (true) {
        uint8_t goBit = drv.readRegister8(DRV2605_REG_GO);
        if ((goBit & 0x01) == 0) break; // Done
        if (millis() - start > 2000) {
            Serial.println("Haptic: Auto-Cal Timeout!");
            break;
        }
        delay(10);
    }

    // 5. Check Status (Diag result)
    uint8_t status = drv.readRegister8(DRV2605_REG_STATUS);
    if (status & 0x08) {
        Serial.println("Haptic: Auto-Cal Failed (Diag Error)");
    } else {
        Serial.println("Haptic: Auto-Cal Complete");
        // Optional: Read back BEMF/Comp results here if we wanted to save them
        // uint8_t comp = drv.readRegister8(DRV2605_REG_A_CAL_COMP);
        // uint8_t bemf = drv.readRegister8(DRV2605_REG_A_CAL_BEMF);
        // Serial.print("Comp: "); Serial.println(comp, HEX);
        // Serial.print("BEMF: "); Serial.println(bemf, HEX);
    }

    // 6. Restore Mode to Internal Trigger
    drv.setMode(DRV2605_MODE_INTTRIG);
}
#endif
