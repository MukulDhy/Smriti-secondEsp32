#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "../config.h"

namespace sensors
{

    struct IMUData
    {
        bool isValid;
        float accelX, accelY, accelZ;
        float gyroX, gyroY, gyroZ;
        float magX, magY, magZ;
        float pressure;
        float temperature;
        float altitude;
        unsigned long timestamp;
    };

    class IMUSensor
    {
    public:
        IMUSensor();
        bool begin();
        IMUData readData();
        bool isConnected() const;
        bool isActive() const;
        void setActive(bool state);
        void calibrate();

    private:
        bool initialized;
        bool active;
        bool magnetometerAvailable;
        bool barometerAvailable;

        float accelOffsetX, accelOffsetY, accelOffsetZ;
        float gyroOffsetX, gyroOffsetY, gyroOffsetZ;

        bool checkMPU6050();
        bool checkMagnetometer();
        bool checkBarometer();
        void initializeMPU6050();
    };

} // namespace sensors

#endif