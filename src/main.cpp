/*
  ESP32-S3 Stable WebSocket Connection with Audio Streaming
  Main Application File
*/

#include <Arduino.h>
#include "config.h"
#include "wifi_manager.h"
#include "websocket_manager.h"
#include "audio_manager.h"
#include "task_manager.h"
#include "utils.h"

void setup()
{
  Serial.begin(115200);
  Serial.println("🚀 ESP32 Stable WebSocket Audio Streaming Client Starting...");

  // Initialize WiFi
  WiFiManager::connect();

  // Initialize Audio System
  AudioManager::init();

  // Initialize WebSocket
  if (WebSocketManager::connect())
  {
    // Start background tasks
    TaskManager::startAllTasks();
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