#ifndef HEART_RATE_SENSOR_H
#define HEART_RATE_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "../config.h"

struct HeartRateData
{
    bool isValid;
    float heartRate;
    float spO2;
    uint32_t irValue;
    uint32_t redValue;
    bool fingerDetected;
    unsigned long timestamp;
};

class HeartRateSensor
{
public:
    HeartRateSensor() : initialized(false), active(true), bufferIndex(0), lastHeartBeat(0), heartRateIndex(0)
    {
        for (int i = 0; i < 4; i++)
        {
            heartRateArray[i] = 0;
        }
    }

    bool begin()
    {
        if (initialized)
            return true;

        Wire.begin(MAX30102_SDA, MAX30102_SCL);

        // Check if sensor is connected
        uint8_t partID = readRegister(0xFF);
        if (partID != 0x15)
        {
            Serial.println("MAX30102 not found");
            return false;
        }

        softReset();
        delay(100);
        setupSensor();

        initialized = true;
        Serial.println("Heart rate sensor initialized");
        return true;
    }

    HeartRateData readData()
    {
        HeartRateData data = {false, 0, 0, 0, 0, false, millis()};

        if (!initialized || !active)
            return data;

        // Read FIFO
        uint8_t readPointer = readRegister(0x06);
        uint8_t writePointer = readRegister(0x04);

        int numberOfSamples = 0;
        if (writePointer != readPointer)
        {
            numberOfSamples = writePointer - readPointer;
            if (numberOfSamples < 0)
                numberOfSamples += 32;
        }

        if (numberOfSamples > 0)
        {
            // Read samples
            Wire.beginTransmission(MAX30102_ADDRESS);
            Wire.write(0x07); // FIFO data register
            Wire.endTransmission();

            Wire.requestFrom(MAX30102_ADDRESS, numberOfSamples * 6);

            while (Wire.available() >= 6 && bufferIndex < HR_BUFFER_SIZE)
            {
                // Read Red LED
                uint32_t redValue = 0;
                redValue |= (uint32_t)Wire.read() << 16;
                redValue |= (uint32_t)Wire.read() << 8;
                redValue |= Wire.read();
                redValue &= 0x3FFFF; // 18-bit data

                // Read IR LED
                uint32_t irValue = 0;
                irValue |= (uint32_t)Wire.read() << 16;
                irValue |= (uint32_t)Wire.read() << 8;
                irValue |= Wire.read();
                irValue &= 0x3FFFF; // 18-bit data

                redBuffer[bufferIndex] = redValue;
                irBuffer[bufferIndex] = irValue;
                bufferIndex = (bufferIndex + 1) % HR_BUFFER_SIZE;

                data.redValue = redValue;
                data.irValue = irValue;
            }

            // Calculate metrics
            data.fingerDetected = detectFinger(data.irValue);

            if (data.fingerDetected)
            {
                data.heartRate = calculateHeartRate(irBuffer, HR_BUFFER_SIZE);
                data.spO2 = calculateSpO2(irBuffer, redBuffer, HR_BUFFER_SIZE);
                data.isValid = true;
            }
        }

        return data;
    }
    uint8_t readRegister(uint8_t reg) const
    {
        Wire.beginTransmission(MAX30102_ADDRESS);
        Wire.write(reg);
        Wire.endTransmission();

        Wire.requestFrom(MAX30102_ADDRESS, 1);
        return Wire.read();
    }
    bool isConnected() const
    {
        return initialized && (readRegister(0xFF) == 0x15);
    }

    bool isActive() const { return active; }

    void setActive(bool state)
    {
        active = state;
        if (initialized)
        {
            if (active)
            {
                setupSensor(); // Reconfigure when activating
            }
            else
            {
                // Power down sensor
                writeRegister(0x09, 0x40); // Reset
                writeRegister(0x0C, 0x00); // Red LED off
                writeRegister(0x0D, 0x00); // IR LED off
            }
        }
    }

    void calibrate()
    {
        Serial.println("Calibrating heart rate sensor...");
        for (int i = 0; i < HR_BUFFER_SIZE; i++)
        {
            irBuffer[i] = 0;
            redBuffer[i] = 0;
        }
        for (int i = 0; i < 4; i++)
        {
            heartRateArray[i] = 0;
        }
        bufferIndex = 0;
        heartRateIndex = 0;
        Serial.println("Heart rate sensor calibration complete");
    }

    void setLEDBrightness(uint8_t brightness)
    {
        if (initialized && active)
        {
            writeRegister(0x0C, brightness); // Red LED
            writeRegister(0x0D, brightness); // IR LED
        }
    }

private:
    bool initialized;
    bool active;
    uint32_t irBuffer[HR_BUFFER_SIZE];
    uint32_t redBuffer[HR_BUFFER_SIZE];
    int bufferIndex;
    unsigned long lastHeartBeat;
    float heartRateArray[4];
    int heartRateIndex;

    bool writeRegister(uint8_t reg, uint8_t value)
    {
        Wire.beginTransmission(MAX30102_ADDRESS);
        Wire.write(reg);
        Wire.write(value);
        return Wire.endTransmission() == 0;
    }

    uint8_t readRegister(uint8_t reg)
    {
        Wire.beginTransmission(MAX30102_ADDRESS);
        Wire.write(reg);
        Wire.endTransmission();

        Wire.requestFrom(MAX30102_ADDRESS, 1);
        return Wire.read();
    }

    void softReset()
    {
        writeRegister(0x09, 0x40); // Reset bit
        while (readRegister(0x09) & 0x40)
            delay(1);
    }

    void setupSensor()
    {
        // FIFO Configuration
        writeRegister(0x08, 0x4F); // Sample averaging: 4, FIFO rollover enabled

        // Mode Configuration
        writeRegister(0x09, 0x03); // SpO2 mode

        // SpO2 Configuration
        writeRegister(0x0A, 0x27); // 100Hz, 411μs pulse width

        // LED Power
        writeRegister(0x0C, 0x24); // Red LED 7.2mA
        writeRegister(0x0D, 0x24); // IR LED 7.2mA

        // Proximity mode
        writeRegister(0x10, 0x7F); // Max power
    }

    float calculateHeartRate(uint32_t *irBuffer, int bufferLength)
    {
        static uint32_t lastValue = 0;
        static bool risingEdge = false;
        static unsigned long lastPeak = 0;

        uint32_t currentValue = irBuffer[(bufferIndex - 1 + bufferLength) % bufferLength];

        // Detect peaks
        if (currentValue > lastValue && !risingEdge)
        {
            risingEdge = true;
        }
        else if (currentValue < lastValue && risingEdge)
        {
            risingEdge = false;

            unsigned long currentTime = millis();
            if (lastPeak > 0)
            {
                float bpm = 60000.0 / (currentTime - lastPeak);
                if (bpm > 50 && bpm < 200)
                {
                    heartRateArray[heartRateIndex] = bpm;
                    heartRateIndex = (heartRateIndex + 1) % 4;

                    float sum = 0;
                    for (int i = 0; i < 4; i++)
                        sum += heartRateArray[i];
                    lastPeak = currentTime;
                    return sum / 4.0;
                }
            }
            lastPeak = currentTime;
        }

        lastValue = currentValue;

        // Return moving average
        float sum = 0;
        for (int i = 0; i < 4; i++)
            sum += heartRateArray[i];
        return sum / 4.0;
    }

    float calculateSpO2(uint32_t *irBuffer, uint32_t *redBuffer, int bufferLength)
    {
        float redAC = 0, redDC = 0, irAC = 0, irDC = 0;

        // Calculate DC components
        for (int i = 0; i < bufferLength; i++)
        {
            redDC += redBuffer[i];
            irDC += irBuffer[i];
        }
        redDC /= bufferLength;
        irDC /= bufferLength;

        // Calculate AC components
        for (int i = 0; i < bufferLength; i++)
        {
            redAC += abs(redBuffer[i] - redDC);
            irAC += abs(irBuffer[i] - irDC);
        }
        redAC /= bufferLength;
        irAC /= bufferLength;

        // Calculate R value and SpO2
        float R = (redAC / redDC) / (irAC / irDC);
        float spO2 = 110 - 25 * R;

        // Clamp to reasonable range
        return constrain(spO2, 70, 100);
    }

    bool detectFinger(uint32_t irValue)
    {
        return irValue > 50000;
    }
};

#endif