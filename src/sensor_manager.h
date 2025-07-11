#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include <driver/i2s.h>
#include <Wire.h>
// Forward declarations for sensor libraries
// class MAX30105;
// class MPU6050;


#include "MPU6050.h" // Add this include
#include "MAX30105.h"

enum SensorType
{
    SENSOR_AUDIO = 0,
    SENSOR_HEART_RATE = 1,
    SENSOR_GYROSCOPE = 2,
    SENSOR_COUNT = 3
};

class SensorManager
{
private:
    // Audio (INMP441)
    static bool audioActive;
    static int16_t audioBuffer[1024];

    // Heart Rate (MAX30102)
    static bool heartRateActive;
    static MAX30105 *heartRateSensor;
    static TwoWire *heartRateWire;
    static float heartRate;
    static float spO2;
    static unsigned long lastHeartRateRead;

    // Gyroscope (GY-87 - MPU6050)
    static bool gyroscopeActive;
    static MPU6050 *gyroscopeSensor;
    static TwoWire *gyroscopeWire;
    static float accelX, accelY, accelZ;
    static float gyroX, gyroY, gyroZ;
    static float temperature;
    static unsigned long lastGyroscopeRead;

    // Sensor status array
    static bool sensorStatus[SENSOR_COUNT];

    // Private initialization methods
    static bool initAudio();
    static bool initHeartRate();
    static bool initGyroscope();

public:
    // General sensor management
    static void init();
    static void startSensor(SensorType sensor);
    static void stopSensor(SensorType sensor);
    static bool isSensorActive(SensorType sensor);
    static void stopAllSensors();

    // Audio methods
    static bool isAudioActive();
    static int16_t *getAudioBuffer();
    static size_t getAudioBufferSize();
    static bool readAudioData(size_t *bytesRead);

    // Heart Rate methods
    static bool isHeartRateActive();
    static bool readHeartRateData();
    static float getHeartRate();
    static float getSpO2();

    // Gyroscope methods
    static bool isGyroscopeActive();
    static bool readGyroscopeData();
    static void getAcceleration(float *x, float *y, float *z);
    static void getRotation(float *x, float *y, float *z);
    static float getTemperature();

    // Utility methods
    static void printSensorStatus();
    static String getSensorStatusJson();
};

#endif