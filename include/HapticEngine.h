#ifndef HAPTIC_ENGINE_H
#define HAPTIC_ENGINE_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

#ifdef ENABLE_DRV2605
#include <Adafruit_DRV2605.h>
#endif

enum HapticEffect {
    EFFECT_NONE = 0,
    EFFECT_STRONG_CLICK = 1,
    EFFECT_SHARP_TICK = 1,
    EFFECT_SOFT_BUMP = 15,
    EFFECT_DOUBLE_CLICK = 10,
    EFFECT_TRIPLE_CLICK = 12,
    EFFECT_BUZZ = 47,
    EFFECT_PULSING_SHARP = 47,
    EFFECT_RAMP_UP = 118,
    EFFECT_RAMP_DOWN = 119,
    EFFECT_TRANSITION_RAMP = 58,
    EFFECT_FALLING = 74
};

class HapticEngine {
public:
    HapticEngine();
    bool begin(TwoWire *wirePrimary, TwoWire *wireSecondary = nullptr);
    
    // Core Actions
    void stop();
    void playEffect(HapticEffect effect, int direction = 0); // 0=Center, -1=Left, 1=Right
    void playPattern(int mode, int intensity, int interval, int direction);
    
    // Specific Notifications
    void playStartup();
    void playShutdown();
    void playBatteryLevel(float voltage);
    void playCalibrationSuccess();
    void playFindMe();
    
    // Direct Control
    void setWaveform(uint8_t slot, uint8_t effect);
    void go();

private:
    bool _beat1 = false;
    bool _beat2 = false;
    unsigned long _lastTrigger;
    int _lastMode;
    #ifdef ENABLE_STEREO_HAPTICS
    Adafruit_DRV2605 _drvSec;
    bool _stereoEnabled;
    #endif
    
    void runAutoCalibration(Adafruit_DRV2605 &drv);
    #endif

    unsigned long _lastTrigger;
    int _lastMode;
};

#endif
