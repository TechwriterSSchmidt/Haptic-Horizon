#ifndef SMART_TERRAIN_H
#define SMART_TERRAIN_H

#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <Adafruit_MLX90640.h>
#include "config.h"

// Thermal Classification Codes
#define THERMAL_NONE 0
#define THERMAL_SMALL 1
#define THERMAL_HUMAN 2
#define THERMAL_MACHINE 3
#define THERMAL_DANGER 4

class SmartTerrain {
public:
    SmartTerrain();
    bool begin(TwoWire *wirePrimary, TwoWire *wireSecondary);
    void stop(); // For power saving
    void start();

    // Main Processing Loop
    void update(float pitch);
    
    // Getters for Haptic Engine
    int getHapticMode();
    int getHapticInterval();
    int getWallDirection(); // -1 Left, 0 Center, 1 Right
    
    // Thermal Data
    void updateThermal();
    int getThermalClass();
    int getThermalCenter(); // 0-31
    int getThermalDist();

    // Raw Data Access (if needed)
    bool isDataReady();

private:
    SparkFun_VL53L5CX _sensor;
    VL53L5CX_ResultsData _measurementData;
    Adafruit_MLX90640 _mlx;
    float _mlxPixels[768];

    // State Variables
    int _hapticMode;
    int _hapticInterval;
    int _wallDirection;
    
    int _thermalClass;
    int _thermalCenter;
    int _thermalDist;
    int _thermalWidth;
};

#endif
