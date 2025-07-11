#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// I2S Configuration (INMP441 Microphone)
#define I2S_SD 13
#define I2S_SCK 14
#define I2S_WS 15
#define I2S_PORT I2S_NUM_0
#define BUFFER_COUNT 10
#define BUFFER_LEN 1024

// I2C Configuration for Heart Rate Sensor (MAX30102)
#define HR_SDA 21
#define HR_SCL 22
#define HR_I2C_FREQ 400000

// I2C Configuration for Gyroscope (GY-87)
#define GYRO_SDA 18
#define GYRO_SCL 19
#define GYRO_I2C_FREQ 400000

// Sensor Data Intervals (in milliseconds)
#define HEART_RATE_INTERVAL 100    // 10 Hz for heart rate
#define GYROSCOPE_INTERVAL 50      // 20 Hz for gyroscope
#define AUDIO_SAMPLE_RATE 16000    // 16 kHz for audio

// WiFi Configuration
extern const char *WIFI_SSID;
extern const char *WIFI_PASSWORD;

// WebSocket Configuration
extern const char *WEBSOCKET_HOST;
extern const uint16_t WEBSOCKET_PORT;
extern const char *WEBSOCKET_PATH;

// Timing Configuration
extern const unsigned long SENSOR_READ_INTERVAL;
extern const unsigned long STATUS_SEND_INTERVAL;
extern const unsigned long PING_INTERVAL;
extern const unsigned long PING_TIMEOUT;
extern const unsigned long RECONNECT_INTERVAL;
extern const int MAX_RECONNECT_ATTEMPTS;

// Device Information
extern const char *DEVICE_NAME;
extern const char *FIRMWARE_VERSION;

#endif