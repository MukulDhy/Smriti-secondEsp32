/*
  ESP32-S3 WebSocket Connection - Alternative Non-SSL Version
  Use this if SSL connections are problematic
*/

#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

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

// Try WS (non-secure) first - many render.com apps support both
const char *websocket_server_url = "ws://simiriti-backend.onrender.com/esp32";
// Backup SSL URL if needed
const char *websocket_server_url_ssl = "wss://simiriti-backend.onrender.com/esp32";

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
const unsigned long sensorReadInterval = 10000;  // 10 seconds (increased)
const unsigned long statusSendInterval = 30000; // 30 seconds (increased)
const unsigned long pingInterval = 45000;       // Send ping every 45 seconds
const unsigned long pingTimeout = 120000;       // 2 minutes timeout

// Connection management
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 20000; // 20 seconds between attempts
int reconnectAttempts = 0;
const int maxReconnectAttempts = 3;

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
bool testInternetConnection();

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
    delay(3000); // Increased delay for slower connections
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

bool testInternetConnection() {
  HTTPClient http;
  http.begin("http://httpbin.org/ip"); // Simple HTTP test
  http.setTimeout(10000); // 10 second timeout
  
  int httpCode = http.GET();
  http.end();
  
  if (httpCode > 0) {
    Serial.printf("✓ Internet connection test passed (HTTP %d)\n", httpCode);
    return true;
  } else {
    Serial.printf("❌ Internet connection test failed (Error: %d)\n", httpCode);
    return false;
  }
}

void connectWiFi()
{
  Serial.println("🔄 Connecting to WiFi...");
  
  // Complete WiFi reset
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(1000);
  
  // Configure WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  
  // Multiple DNS servers for redundancy
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, 
              IPAddress(8, 8, 8, 8), IPAddress(1, 1, 1, 1));
  
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 50)
  {
    delay(500);
    Serial.print(".");
    attempts++;
    
    // Try reconnecting every 10 attempts
    if (attempts % 10 == 0) {
      Serial.println();
      Serial.printf("🔄 WiFi attempt %d/50\n", attempts);
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(ssid, password);
    }
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\n✓ WiFi connected!");
    Serial.print("📍 IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("📶 Signal Strength: ");
    Serial.println(WiFi.RSSI());
    Serial.print("🌐 Gateway: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("🔍 DNS: ");
    Serial.println(WiFi.dnsIP());
    
    // Test internet connectivity
    delay(2000);
    if (!testInternetConnection()) {
      Serial.println("⚠️ Internet connection issues detected");
    }
    
    // Test DNS resolution
    Serial.println("🔍 Testing DNS resolution...");
    IPAddress serverIP;
    if (WiFi.hostByName("simiriti-backend.onrender.com", serverIP)) {
      Serial.print("✓ DNS resolved: ");
      Serial.println(serverIP);
    } else {
      Serial.println("❌ DNS resolution failed - trying alternative DNS");
      WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, 
                  IPAddress(1, 1, 1, 1), IPAddress(8, 8, 4, 4));
      delay(2000);
      if (WiFi.hostByName("simiriti-backend.onrender.com", serverIP)) {
        Serial.print("✓ DNS resolved with alternative DNS: ");
        Serial.println(serverIP);
      } else {
        Serial.println("❌ DNS still failing");
      }
    }
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
    Serial.println("❌ WiFi not connected");
    return false;
  }

  Serial.println("🔄 Connecting to WebSocket server...");
  
  // Clean up existing connection
  if (isWebSocketConnected) {
    client.close();
    delay(3000);
  }
  
  // Test DNS resolution first
  IPAddress serverIP;
  if (!WiFi.hostByName("simiriti-backend.onrender.com", serverIP)) {
    Serial.println("❌ Cannot resolve server hostname");
    return false;
  }
  Serial.print("✓ Server IP resolved: ");
  Serial.println(serverIP);
  
  client.onEvent(onEventsCallback);
  client.onMessage(onMessageCallback);
  
  bool connected = false;
  
  // Try multiple connection methods
  for (int attempt = 1; attempt <= 4; attempt++) {
    Serial.printf("🔄 Connection attempt %d/4\n", attempt);
    
    switch(attempt) {
      case 1:
        // Method 1: Direct WS connection (non-secure)
        Serial.println("Trying: WS (non-secure) connection");
        connected = client.connect(websocket_server_url);
        break;
        
      case 2:
        // Method 2: WSS connection
        Serial.println("Trying: WSS (secure) connection");
        connected = client.connect(websocket_server_url_ssl);
        break;
        
      case 3:
        // Method 3: Connect using IP address with WS
        {
          String ipWsUrl = "ws://" + serverIP.toString() + "/esp32";
          Serial.printf("Trying: IP-based WS connection (%s)\n", ipWsUrl.c_str());
          connected = client.connect(ipWsUrl);
        }
        break;
        
      case 4:
        // Method 4: Explicit host and port
        Serial.println("Trying: Explicit host/port connection");
        connected = client.connect("simiriti-backend.onrender.com", 80, "/esp32");
        break;
    }

    if (connected) {
      Serial.printf("✓ WebSocket Connected using method %d!\n", attempt);
      break;
    } else {
      Serial.printf("❌ Method %d failed, waiting before retry...\n", attempt);
      delay(5000);
    }
  }

  if (connected)
  {
    lastPingReceived = millis();
    lastPingSent = millis();
    lastConnectionAttempt = millis();
    return true;
  }
  else
  {
    Serial.println("❌ All WebSocket connection methods failed");
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
  doc["firmwareVersion"] = "1.0.5";
  doc["deviceId"] = "ESP32-" + String(ESP.getEfuseMac(), HEX);

  JsonArray capabilities = doc["capabilities"].to<JsonArray>();
  capabilities.add("audio");
  capabilities.add("sensors");

  JsonArray sensorTypes = doc["sensorTypes"].to<JsonArray>();
  sensorTypes.add("audio");
  sensorTypes.add("system");

  doc["batteryLevel"] = 85;
  doc["signalStrength"] = WiFi.RSSI();
  doc["timestamp"] = millis();
  doc["freeHeap"] = ESP.getFreeHeap();
  doc["chipModel"] = ESP.getChipModel();
  doc["macAddress"] = WiFi.macAddress();

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
  doc["deviceId"] = "ESP32-" + String(ESP.getEfuseMac(), HEX);

  JsonObject data = doc["data"].to<JsonObject>();
  data["freeHeap"] = ESP.getFreeHeap();
  data["rssi"] = WiFi.RSSI();
  data["uptime"] = millis();
  data["audioStreaming"] = isAudioStreaming;
  data["wifiStatus"] = WiFi.status();
  data["temperature"] = temperatureRead();

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
  doc["deviceId"] = "ESP32-" + String(ESP.getEfuseMac(), HEX);

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
  doc["deviceId"] = "ESP32-" + String(ESP.getEfuseMac(), HEX);

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
    doc["deviceId"] = "ESP32-" + String(ESP.getEfuseMac(), HEX);

    String message;
    serializeJson(doc, message);
    client.send(message);
    delay(5000);
  }

  ESP.restart();
}

void micTask(void *parameter)
{
  Serial.println("🎤 Starting microphone task...");

  size_t bytesIn = 0;
  int consecutiveErrors = 0;
  const int maxConsecutiveErrors = 100; // Increased tolerance

  while (true)
  {
    if (isWebSocketConnected && isAudioStreaming && connectionEstablished)
    {
      esp_err_t result = i2s_read(I2S_PORT, &sBuffer, BUFFER_LEN * sizeof(int16_t), &bytesIn, 200);

      if (result == ESP_OK && bytesIn > 0)
      {
        if (isWebSocketConnected && client.available()) {
          bool sent = client.sendBinary((const char *)sBuffer, bytesIn);
          if (!sent) {
            Serial.println("⚠️ Failed to send audio data");
          }
          consecutiveErrors = 0;
        }
      }
      else
      {
        consecutiveErrors++;
        if (consecutiveErrors % 50 == 0)
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
      vTaskDelay(200 / portTICK_PERIOD_MS);
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

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("🚀 ESP32 WebSocket Audio Client Starting (Alternative Version)...");
  
  Serial.printf("📋 Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("📋 Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("📋 Free Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("📋 Flash Size: %d bytes\n", ESP.getFlashChipSize());

  connectWiFi();
  setupI2S();

  delay(5000);

  if (connectWebSocket())
  {
    xTaskCreatePinnedToCore(
        micTask,
        "micTask",
        12000,
        NULL,
        2,
        &micTaskHandle,
        1);

    xTaskCreatePinnedToCore(
        sensorTask,
        "sensorTask",
        10000,
        NULL,
        1,
        &sensorTaskHandle,
        0);
  }
  else
  {
    Serial.println("❌ Failed to connect to WebSocket server - will retry in loop");
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

    // Send periodic ping
    if (now - lastPingSent >= pingInterval && connectionEstablished)
    {
      sendPing();
    }

    // Check for ping timeout
    if (now - lastPingReceived > pingTimeout)
    {
      Serial.println("❌ Ping timeout - connection may be lost");
      isWebSocketConnected = false;
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
    Serial.println("❌ WiFi disconnected - attempting reconnection");
    connectWiFi();
  }

  delay(250); // Slightly increased delay for better stability
}