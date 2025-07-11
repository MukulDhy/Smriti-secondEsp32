/*
  ESP32-S3 Stable WebSocket Connection with Multi-Sensor Support
  Main Application File
*/

#include <Arduino.h>
#include "config.h"
#include "wifi_manager.h"
#include "websocket_manager.h"
#include "sensor_manager.h"
#include "task_manager.h"
#include "utils.h"

void setup()
{
  Serial.begin(115200);
  Serial.println("🚀 ESP32 Multi-Sensor WebSocket Client Starting...");

  // Initialize WiFi
  WiFiManager::connect();

  // Initialize All Sensors (but don't start them)
  SensorManager::init();

  // Initialize WebSocket
  if (WebSocketManager::connect())
  {
    // Start background tasks (sensor task for periodic updates)
    TaskManager::startAllTasks();
    
    Serial.println("✓ System initialized successfully");
    Serial.println("📡 Waiting for backend commands to start sensors...");
  }
  else
  {
    Serial.println("❌ Failed to connect to WebSocket server - restarting");
    Utils::restartSystem("Initial WebSocket Connection Failed");
  }
}

void loop()
{
  unsigned long now = millis();

  if (WebSocketManager::isConnected())
  {
    // Poll WebSocket for messages
    WebSocketManager::poll();

    // Send periodic status updates
    WebSocketManager::handlePeriodicUpdates(now);
  }
  else
  {
    // Handle reconnection logic
    WebSocketManager::handleReconnection(now);
  }

  // Check WiFi connection
  if (!WiFiManager::isConnected())
  {
    Serial.println("❌ WiFi disconnected - restarting");
    Utils::restartSystem("WiFi Disconnected");
  }

  delay(100);
}