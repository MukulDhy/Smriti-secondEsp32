#include "config.h"

// WiFi Configuration
const char *WIFI_SSID = "Mukuldhy";
const char *WIFI_PASSWORD = "12345678";

// WebSocket Configuration
const char *WEBSOCKET_HOST = "192.168.0.103";
const uint16_t WEBSOCKET_PORT = 5000;
const char *WEBSOCKET_PATH = "/esp32";

// Timing Configuration
const unsigned long SENSOR_READ_INTERVAL = 5000;  // 5 seconds
const unsigned long STATUS_SEND_INTERVAL = 15000; // 15 seconds
const unsigned long PING_INTERVAL = 30000;        // 30 seconds
const unsigned long PING_TIMEOUT = 45000;         // 45 seconds
const unsigned long RECONNECT_INTERVAL = 5000;    // 5 seconds
const int MAX_RECONNECT_ATTEMPTS = 15;

// Device Information
const char *DEVICE_NAME = "ESP32-Audio-Sensor";
const char *FIRMWARE_VERSION = "1.0.2";