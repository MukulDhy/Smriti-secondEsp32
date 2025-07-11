#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// I2S Configuration
#define I2S_SD 13
#define I2S_SCK 14
#define I2S_WS 15
#define I2S_PORT I2S_NUM_0
#define BUFFER_COUNT 10
#define BUFFER_LEN 1024

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