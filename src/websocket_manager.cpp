#include "websocket_manager.h"
#include "config.h"
#include "wifi_manager.h"
#include "sensor_manager.h"
#include "task_manager.h"
#include "utils.h"

// Static member definitions
WebsocketsClient WebSocketManager::client;
bool WebSocketManager::isWebSocketConnected = false;
bool WebSocketManager::connectionEstablished = false;
bool WebSocketManager::deviceInfoSent = false;
unsigned long WebSocketManager::lastStatusSend = 0;
unsigned long WebSocketManager::lastPingReceived = 0;
unsigned long WebSocketManager::lastPingSent = 0;
unsigned long WebSocketManager::lastReconnectAttempt = 0;
int WebSocketManager::reconnectAttempts = 0;

void WebSocketManager::onEventsCallback(WebsocketsEvent event, String data)
{
  switch (event)
  {
  case WebsocketsEvent::ConnectionOpened:
    Serial.println("✓ WebSocket Connection Opened");
    isWebSocketConnected = true;
    connectionEstablished = false;
    deviceInfoSent = false;
    reconnectAttempts = 0;
    lastPingReceived = millis();
    lastPingSent = millis();

    delay(500);
    sendDeviceInfo();
    break;

  case WebsocketsEvent::ConnectionClosed:
    Serial.println("✗ WebSocket Connection Closed");
    isWebSocketConnected = false;
    connectionEstablished = false;
    deviceInfoSent = false;
    // Stop all sensors when connection is lost
    SensorManager::stopAllSensors();
    TaskManager::stopAllTasks();
    break;

  case WebsocketsEvent::GotPing:
    Serial.println("📡 Got Ping from server - auto pong sent");
    lastPingReceived = millis();
    break;

  case WebsocketsEvent::GotPong:
    Serial.println("📡 Got Pong from server");
    lastPingReceived = millis();
    break;
  }
}

void WebSocketManager::onMessageCallback(WebsocketsMessage message)
{
  Serial.print("📨 Got Message: ");
  Serial.println(message.data());

  lastPingReceived = millis();

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message.data());

  if (error)
  {
    Serial.println("❌ JSON parsing failed");
    return;
  }

  String messageType = doc["type"];

  if (messageType == "connection-established")
  {
    Serial.println("✓ ESP32 connection confirmed by backend");
    connectionEstablished = true;
  }
  else if (messageType == "command")
  {
    String command = doc["command"];
    
    if (command == "start-audio-stream")
    {
      SensorManager::startSensor(SENSOR_AUDIO);
      TaskManager::startAudioTask();
    }
    else if (command == "stop-audio-stream")
    {
      SensorManager::stopSensor(SENSOR_AUDIO);
      TaskManager::stopAudioTask();
    }
    else if (command == "start-heart-rate")
    {
      SensorManager::startSensor(SENSOR_HEART_RATE);
      TaskManager::startHeartRateTask();
    }
    else if (command == "stop-heart-rate")
    {
      SensorManager::stopSensor(SENSOR_HEART_RATE);
      TaskManager::stopHeartRateTask();
    }
    else if (command == "start-gyroscope")
    {
      SensorManager::startSensor(SENSOR_GYROSCOPE);
      TaskManager::startGyroscopeTask();
    }
    else if (command == "stop-gyroscope")
    {
      SensorManager::stopSensor(SENSOR_GYROSCOPE);
      TaskManager::stopGyroscopeTask();
    }
    else if (command == "get-sensor-data")
    {
      sendSensorData();
    }
    else if (command == "get-sensor-status")
    {
      sendSensorStatus();
    }
    else if (command == "stop-all-sensors")
    {
      SensorManager::stopAllSensors();
      TaskManager::stopAudioTask();
      TaskManager::stopHeartRateTask();
      TaskManager::stopGyroscopeTask();
    }
  }
  else if (messageType == "ping")
  {
    JsonDocument response;
    response["type"] = "pong";
    response["timestamp"] = millis();

    String pongMessage;
    serializeJson(response, pongMessage);
    client.send(pongMessage);
    Serial.println("📡 Sent pong response to server ping");
  }
  else if (messageType == "pong")
  {
    Serial.println("📡 Received pong from server");
    lastPingReceived = millis();
  }
}

bool WebSocketManager::connect()
{
  if (!WiFiManager::isConnected())
  {
    return false;
  }

  Serial.println("🔄 Connecting to WebSocket server...");
  client.onEvent(onEventsCallback);
  client.onMessage(onMessageCallback);

  bool connected = client.connect(WEBSOCKET_HOST, WEBSOCKET_PORT, WEBSOCKET_PATH);

  if (connected)
  {
    Serial.println("✓ WebSocket Connected!");
    lastPingReceived = millis();
    lastPingSent = millis();
    return true;
  }
  else
  {
    Serial.println("❌ WebSocket connection failed");
    return false;
  }
}

void WebSocketManager::sendDeviceInfo()
{
  if (!isWebSocketConnected || deviceInfoSent)
    return;

  JsonDocument doc;
  doc["type"] = "device-info";
  doc["deviceName"] = DEVICE_NAME;
  doc["firmwareVersion"] = FIRMWARE_VERSION;

  JsonArray capabilities = doc["capabilities"].to<JsonArray>();
  capabilities.add("audio");
  capabilities.add("heartRate");
  capabilities.add("gyroscope");

  JsonArray sensorTypes = doc["sensorTypes"].to<JsonArray>();
  sensorTypes.add("audio");
  sensorTypes.add("heartRate");
  sensorTypes.add("gyroscope");

  doc["batteryLevel"] = 85;
  doc["signalStrength"] = WiFiManager::getSignalStrength();
  doc["timestamp"] = millis();

  String message;
  serializeJson(doc, message);

  if (client.send(message))
  {
    deviceInfoSent = true;
    Serial.println("✓ Device info sent to backend");
  }
  else
  {
    Serial.println("❌ Failed to send device info");
  }
}

void WebSocketManager::sendSensorData()
{
  if (!isWebSocketConnected || !connectionEstablished)
    return;

  JsonDocument doc;
  doc["type"] = "sensor-data";
  doc["sensorType"] = "system";
  doc["timestamp"] = millis();

  JsonObject data = doc["data"].to<JsonObject>();
  data["freeHeap"] = ESP.getFreeHeap();
  data["rssi"] = WiFiManager::getSignalStrength();
  data["uptime"] = millis();
  data["audioActive"] = SensorManager::isAudioActive();
  data["heartRateActive"] = SensorManager::isHeartRateActive();
  data["gyroscopeActive"] = SensorManager::isGyroscopeActive();

  String message;
  serializeJson(doc, message);

  if (client.send(message))
  {
    Serial.println("📊 Sensor data sent");
  }
}

void WebSocketManager::sendSensorStatus()
{
  if (!isWebSocketConnected || !connectionEstablished)
    return;

  JsonDocument doc;
  doc["type"] = "sensor-status";
  doc["timestamp"] = millis();

  JsonObject sensors = doc["sensors"].to<JsonObject>();
  sensors["audio"] = SensorManager::isAudioActive();
  sensors["heartRate"] = SensorManager::isHeartRateActive();
  sensors["gyroscope"] = SensorManager::isGyroscopeActive();

  String message;
  serializeJson(doc, message);

  if (client.send(message))
  {
    Serial.println("📊 Sensor status sent");
  }
}

void WebSocketManager::sendStatusUpdate()
{
  if (!isWebSocketConnected || !connectionEstablished)
    return;

  JsonDocument doc;
  doc["type"] = "status-update";
  doc["status"] = "online";
  doc["sensors"] = SensorManager::getSensorStatusJson();
  doc["batteryLevel"] = 85;
  doc["signalStrength"] = WiFiManager::getSignalStrength();
  doc["timestamp"] = millis();
  doc["uptime"] = millis();
  doc["freeHeap"] = ESP.getFreeHeap();

  String message;
  serializeJson(doc, message);

  if (client.send(message))
  {
    Serial.println("✓ Status update sent");
  }
}

void WebSocketManager::sendPing()
{
  if (!isWebSocketConnected)
    return;

  JsonDocument doc;
  doc["type"] = "ping";
  doc["timestamp"] = millis();

  String message;
  serializeJson(doc, message);

  if (client.send(message))
  {
    lastPingSent = millis();
    Serial.println("📡 Sent ping to server");
  }
}

bool WebSocketManager::isConnected()
{
  return isWebSocketConnected;
}

bool WebSocketManager::isEstablished()
{
  return connectionEstablished;
}

void WebSocketManager::poll()
{
  client.poll();
}

void WebSocketManager::handlePeriodicUpdates(unsigned long now)
{
  if (now - lastStatusSend >= STATUS_SEND_INTERVAL && connectionEstablished)
  {
    lastStatusSend = now;
    sendStatusUpdate();
  }
}

void WebSocketManager::handleReconnection(unsigned long now)
{
  if (now - lastReconnectAttempt > RECONNECT_INTERVAL && reconnectAttempts < MAX_RECONNECT_ATTEMPTS)
  {
    lastReconnectAttempt = now;
    reconnectAttempts++;

    Serial.printf("🔄 Attempting to reconnect... (%d/%d)\n", reconnectAttempts, MAX_RECONNECT_ATTEMPTS);

    if (connect())
    {
      Serial.println("✓ Reconnected successfully!");
    }
  }
  else if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS)
  {
    Serial.println("❌ Max reconnect attempts reached - restarting");
    Utils::restartSystem("Max Reconnect Attempts Reached");
  }
}

void WebSocketManager::sendBinary(const char* data, size_t length)
{
  if (isWebSocketConnected && connectionEstablished)
  {
    client.sendBinary(data, length);
  }
}

void WebSocketManager::sendText(const String& message)
{
  if (isWebSocketConnected && connectionEstablished)
  {
    client.send(message);
  }
}