/*
  ESP32-S3 Stable WebSocket Connection with Audio Streaming
  Fixed Version - Optimized Connection Management
*/

#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>

using namespace websockets;

// I2S Configuration
#define I2S_SD 13
#define I2S_SCK 14
#define I2S_WS 15
#define I2S_PORT I2S_NUM_0

#define BUFFER_COUNT 10
#define BUFFER_LEN 1024
int16_t sBuffer[BUFFER_LEN];

// WiFi & WebSocket Config
const char *ssid = "Mukuldhy";
const char *password = "12345678";

const char *websocket_server_host = "192.168.0.103";
const uint16_t websocket_server_port = 5000;
const char *websocket_path = "/esp32";

WebsocketsClient client;
bool isWebSocketConnected = false;
bool isAudioStreaming = false;

// Connection timing
unsigned long lastSensorRead = 0;
unsigned long lastStatusSend = 0;
unsigned long lastPingReceived = 0;
unsigned long lastPingSent = 0;
unsigned long lastConnectionAttempt = 0;

// Optimized timing intervals
const unsigned long sensorReadInterval = 5000;  // 5 seconds
const unsigned long statusSendInterval = 15000; // 15 seconds
const unsigned long pingInterval = 30000;       // Send ping every 30 seconds
const unsigned long pingTimeout = 45000;        // 45 seconds timeout

// Connection management
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 5000; // 5 seconds between attempts
int reconnectAttempts = 0;
const int maxReconnectAttempts = 15; // Increased max attempts

// Connection state tracking
bool connectionEstablished = false;
bool deviceInfoSent = false;

// Task handles
TaskHandle_t micTaskHandle = nullptr;
TaskHandle_t sensorTaskHandle = nullptr;

// Forward declarations
void sendDeviceInfo();
void sendStatusUpdate();
void startAudioStreaming();
void stopAudioStreaming();
void sendSensorData();
void sendPing();
void restartSystem(const char *reason);

void onEventsCallback(WebsocketsEvent event, String data)
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
    lastConnectionAttempt = millis();

    // Send device info after short delay
    delay(500);
    sendDeviceInfo();
    break;

  case WebsocketsEvent::ConnectionClosed:
    Serial.println("✗ WebSocket Connection Closed");
    isWebSocketConnected = false;
    isAudioStreaming = false;
    connectionEstablished = false;
    deviceInfoSent = false;
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

void onMessageCallback(WebsocketsMessage message)
{
  Serial.print("📨 Got Message: ");
  Serial.println(message.data());

  lastPingReceived = millis(); // Any message counts as activity

  // Parse JSON message
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
    startAudioStreaming();
  }
  else if (messageType == "command")
  {
    String command = doc["command"];
    if (command == "start-audio-stream")
    {
      startAudioStreaming();
    }
    else if (command == "stop-audio-stream")
    {
      stopAudioStreaming();
    }
    else if (command == "get-sensor-data")
    {
      sendSensorData();
    }
  }
  else if (messageType == "ping")
  {
    // Respond to server ping
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

void setupI2S()
{
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = BUFFER_COUNT,
      .dma_buf_len = BUFFER_LEN,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0};

  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = -1,
      .data_in_num = I2S_SD};

  esp_err_t result = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (result != ESP_OK)
  {
    Serial.printf("❌ I2S install failed: %d\n", result);
    return;
  }

  result = i2s_set_pin(I2S_PORT, &pin_config);
  if (result != ESP_OK)
  {
    Serial.printf("❌ I2S pin config failed: %d\n", result);
    return;
  }

  result = i2s_set_clk(I2S_PORT, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  if (result != ESP_OK)
  {
    Serial.printf("❌ I2S clock config failed: %d\n", result);
    return;
  }

  result = i2s_start(I2S_PORT);
  if (result != ESP_OK)
  {
    Serial.printf("❌ I2S start failed: %d\n", result);
    return;
  }

  Serial.println("✓ I2S initialized successfully");
}

void connectWiFi()
{
  Serial.println("🔄 Connecting to WiFi...");
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\n✓ WiFi connected!");
    Serial.print("📍 IP Address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("\n❌ WiFi connection failed - restarting");
    restartSystem("WiFi Connection Failed");
  }
}

bool connectWebSocket()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    return false;
  }

  Serial.println("🔄 Connecting to WebSocket server...");
  client.onEvent(onEventsCallback);
  client.onMessage(onMessageCallback);

  // Remove setReconnectInterval() as it's not available in this version
  // client.setReconnectInterval(0); // Remove this line

  // For SSL connections, you would use:
  // client.setInsecure(); // Only if your server doesn't use SSL

  bool connected = client.connect(websocket_server_host, websocket_server_port, websocket_path);

  if (connected)
  {
    Serial.println("✓ WebSocket Connected!");
    lastPingReceived = millis();
    lastPingSent = millis();
    lastConnectionAttempt = millis();
    return true;
  }
  else
  {
    Serial.println("❌ WebSocket connection failed");
    return false;
  }
}

void sendDeviceInfo()
{
  if (!isWebSocketConnected || deviceInfoSent)
    return;

  JsonDocument doc;
  doc["type"] = "device-info";
  doc["deviceName"] = "ESP32-Audio-Sensor";
  doc["firmwareVersion"] = "1.0.2";

  // Updated way to create nested arrays
  JsonArray capabilities = doc["capabilities"].to<JsonArray>();
  capabilities.add("audio");

  JsonArray sensorTypes = doc["sensorTypes"].to<JsonArray>();
  sensorTypes.add("audio");

  doc["batteryLevel"] = 85;
  doc["signalStrength"] = WiFi.RSSI();
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

void sendSensorData()
{
  if (!isWebSocketConnected || !connectionEstablished)
    return;

  JsonDocument doc;
  doc["type"] = "sensor-data";
  doc["sensorType"] = "system";
  doc["timestamp"] = millis();

  // Updated way to create nested objects
  JsonObject data = doc["data"].to<JsonObject>();
  data["freeHeap"] = ESP.getFreeHeap();
  data["rssi"] = WiFi.RSSI();
  data["uptime"] = millis();
  data["audioStreaming"] = isAudioStreaming;

  String message;
  serializeJson(doc, message);

  if (client.send(message))
  {
    Serial.println("📊 Sensor data sent");
  }
}
void sendStatusUpdate()
{
  if (!isWebSocketConnected || !connectionEstablished)
    return;

  JsonDocument doc;
  doc["type"] = "status-update";
  doc["status"] = "online";
  doc["isStreaming"] = isAudioStreaming;
  doc["batteryLevel"] = 85;
  doc["signalStrength"] = WiFi.RSSI();
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

void sendPing()
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

void startAudioStreaming()
{
  if (!isAudioStreaming)
  {
    isAudioStreaming = true;
    Serial.println("🎤 Audio streaming started");
    sendStatusUpdate();
  }
}

void stopAudioStreaming()
{
  if (isAudioStreaming)
  {
    isAudioStreaming = false;
    Serial.println("🎤 Audio streaming stopped");
    sendStatusUpdate();
  }
}

void restartSystem(const char *reason)
{
  Serial.printf("🔄 RESTARTING SYSTEM: %s\n", reason);

  if (isWebSocketConnected)
  {
    JsonDocument doc;
    doc["type"] = "system-restart";
    doc["reason"] = reason;
    doc["timestamp"] = millis();

    String message;
    serializeJson(doc, message);
    client.send(message);
    delay(1000);
  }

  ESP.restart();
}

void micTask(void *parameter)
{
  Serial.println("🎤 Starting microphone task...");

  size_t bytesIn = 0;
  int consecutiveErrors = 0;
  const int maxConsecutiveErrors = 50;

  while (true)
  {
    if (isWebSocketConnected && isAudioStreaming && connectionEstablished)
    {
      esp_err_t result = i2s_read(I2S_PORT, &sBuffer, BUFFER_LEN * sizeof(int16_t), &bytesIn, 100);

      if (result == ESP_OK && bytesIn > 0)
      {
        client.sendBinary((const char *)sBuffer, bytesIn);
        consecutiveErrors = 0;
      }
      else
      {
        consecutiveErrors++;
        if (consecutiveErrors % 20 == 0)
        {
          Serial.printf("⚠️ I2S read error: %d, consecutive errors: %d\n", result, consecutiveErrors);
        }

        if (consecutiveErrors > maxConsecutiveErrors)
        {
          Serial.println("❌ Too many I2S errors - restarting");
          restartSystem("I2S Read Errors");
        }
      }
    }
    else
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

void sensorTask(void *parameter)
{
  Serial.println("📊 Starting sensor task...");

  while (true)
  {
    unsigned long now = millis();

    if (now - lastSensorRead >= sensorReadInterval && connectionEstablished)
    {
      lastSensorRead = now;
      sendSensorData();
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("🚀 ESP32 Stable WebSocket Audio Streaming Client Starting...");

  connectWiFi();
  setupI2S();

  if (connectWebSocket())
  {
    // Start microphone task on core 1
    xTaskCreatePinnedToCore(
        micTask,
        "micTask",
        10000,
        NULL,
        2,
        &micTaskHandle,
        1);

    // Start sensor task on core 0
    xTaskCreatePinnedToCore(
        sensorTask,
        "sensorTask",
        8000,
        NULL,
        1,
        &sensorTaskHandle,
        0);
  }
  else
  {
    Serial.println("❌ Failed to connect to WebSocket server - restarting");
    restartSystem("Initial WebSocket Connection Failed");
  }
}

void loop()
{
  unsigned long now = millis();

  if (isWebSocketConnected)
  {
    // Poll WebSocket for messages
    client.poll();

    // Send periodic status updates
    if (now - lastStatusSend >= statusSendInterval && connectionEstablished)
    {
      lastStatusSend = now;
      sendStatusUpdate();
    }
  }
  else
  {
    // Reconnection logic
    if (now - lastReconnectAttempt > reconnectInterval && reconnectAttempts < maxReconnectAttempts)
    {
      lastReconnectAttempt = now;
      reconnectAttempts++;

      Serial.printf("🔄 Attempting to reconnect... (%d/%d)\n", reconnectAttempts, maxReconnectAttempts);

      if (connectWebSocket())
      {
        Serial.println("✓ Reconnected successfully!");
      }
    }
    else if (reconnectAttempts >= maxReconnectAttempts)
    {
      Serial.println("❌ Max reconnect attempts reached - restarting");
      restartSystem("Max Reconnect Attempts Reached");
    }
  }

  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("❌ WiFi disconnected - restarting");
    restartSystem("WiFi Disconnected");
  }

  delay(100);
}