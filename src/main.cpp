/*
  ESP32-S3 Audio Streaming to WebSocket Server
  Updated for basic WebSocket connection without authentication
*/

#include <driver/i2s.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>

#define I2S_SD 13
#define I2S_SCK 14
#define I2S_WS 15
#define I2S_PORT I2S_NUM_0

#define bufferCnt 10
#define bufferLen 1024
int16_t sBuffer[bufferLen];

// WiFi Configuration
const char *ssid = "Mukuldhy";
const char *password = "12345678";

// WebSocket Server Configuration
const char *websocket_server_host = "192.168.0.103"; // e.g., "192.168.1.100"
const uint16_t websocket_server_port = 5000;         // Your server port
const char *websocket_path = "/esp32-audio";         // Path for ESP32 connections

using namespace websockets;
WebsocketsClient client;
bool isWebSocketConnected = false;
bool isStreaming = false;

// Connection retry variables
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 5000; // 5 seconds
void sendStreamingStatus(bool streaming);
void onEventsCallback(WebsocketsEvent event, String data)
{
  switch (event)
  {
  case WebsocketsEvent::ConnectionOpened:
    Serial.println("WebSocket Connection Opened");
    isWebSocketConnected = true;
    sendStreamingStatus(true);
    break;

  case WebsocketsEvent::ConnectionClosed:
    Serial.println("WebSocket Connection Closed");
    isWebSocketConnected = false;
    isStreaming = false;
    break;

  case WebsocketsEvent::GotPing:
    Serial.println("Got Ping from server");
    break;

  case WebsocketsEvent::GotPong:
    Serial.println("Got Pong from server");
    break;
  }
}

void onMessageCallback(WebsocketsMessage message)
{
  Serial.print("Got Message: ");
  Serial.println(message.data());

  // Handle server messages (optional)
  // You can add message handling here if needed
}

void i2s_install()
{
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = 16000, // Using 16kHz for better performance
      .bits_per_sample = i2s_bits_per_sample_t(16),
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
      .intr_alloc_flags = 0,
      .dma_buf_count = bufferCnt,
      .dma_buf_len = bufferLen,
      .use_apll = false};

  esp_err_t result = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (result != ESP_OK)
  {
    Serial.printf("Failed to install I2S driver: %d\n", result);
  }
  else
  {
    Serial.println("I2S driver installed successfully");
  }
}

void i2s_setpin()
{
  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = -1,
      .data_in_num = I2S_SD};

  esp_err_t result = i2s_set_pin(I2S_PORT, &pin_config);
  if (result != ESP_OK)
  {
    Serial.printf("Failed to set I2S pins: %d\n", result);
  }
  else
  {
    Serial.println("I2S pins configured successfully");
  }
}

void connectWiFi()
{
  Serial.println("Connecting to WiFi...");
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
    Serial.println("");
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("");
    Serial.println("Failed to connect to WiFi. Restarting...");
    ESP.restart();
  }
}

bool connectWSServer()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    return false;
  }

  Serial.println("Connecting to WebSocket server...");
  client.onEvent(onEventsCallback);
  client.onMessage(onMessageCallback);

  String url = String("ws://") + websocket_server_host + ":" + websocket_server_port + websocket_path;
  Serial.printf("Connecting to: %s\n", url.c_str());

  bool connected = client.connect(websocket_server_host, websocket_server_port, websocket_path);

  if (connected)
  {
    Serial.println("WebSocket Connected!");
    return true;
  }
  else
  {
    Serial.println("WebSocket connection failed");
    return false;
  }
}

void sendStreamingStatus(bool streaming)
{
  if (isWebSocketConnected)
  {
    String message = streaming ? "{\"type\":\"start-streaming\"}" : "{\"type\":\"stop-streaming\"}";
    client.send(message);
    isStreaming = streaming;
    Serial.printf("Sent streaming status: %s\n", streaming ? "started" : "stopped");
  }
}

void micTask(void *parameter)
{
  Serial.println("Starting microphone task...");

  i2s_install();
  i2s_setpin();

  esp_err_t result = i2s_start(I2S_PORT);
  if (result != ESP_OK)
  {
    Serial.printf("Failed to start I2S: %d\n", result);
    vTaskDelete(NULL);
    return;
  }

  Serial.println("I2S started, beginning audio capture...");
  size_t bytesIn = 0;
  int consecutiveErrors = 0;

  while (1)
  {
    if (isWebSocketConnected)
    {
      esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen * sizeof(int16_t), &bytesIn, portMAX_DELAY);

      if (result == ESP_OK && bytesIn > 0)
      {
        // Send binary audio data
        client.sendBinary((const char *)sBuffer, bytesIn);
        consecutiveErrors = 0;

        // Optional: Print audio level for debugging
        static int sampleCount = 0;
        if (sampleCount++ % 100 == 0)
        {
          int16_t maxSample = 0;
          for (int i = 0; i < bytesIn / sizeof(int16_t); i++)
          {
            if (abs(sBuffer[i]) > maxSample)
            {
              maxSample = abs(sBuffer[i]);
            }
          }
          Serial.printf("Audio level: %d\n", maxSample);
        }
      }
      else
      {
        consecutiveErrors++;
        Serial.printf("I2S read error: %d, bytes: %d\n", result, bytesIn);

        if (consecutiveErrors > 10)
        {
          Serial.println("Too many I2S errors, restarting...");
          // ESP.restart();
        }
      }
    }
    else
    {
      // Wait when not connected
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("ESP32 Audio Streaming Client Starting...");

  // Connect to WiFi
  connectWiFi();

  // Connect to WebSocket server
  if (connectWSServer())
  {
    // Start microphone task
    xTaskCreatePinnedToCore(
        micTask,   // Task function
        "micTask", // Task name
        10000,     // Stack size
        NULL,      // Parameter
        1,         // Priority
        NULL,      // Task handle
        1          // Core
    );
  }
  else
  {
    Serial.println("Failed to connect to WebSocket server");
  }
}

void loop()
{
  // Handle WebSocket events
  if (isWebSocketConnected)
  {
    client.poll();
  }
  else
  {
    // Try to reconnect
    unsigned long now = millis();
    if (now - lastReconnectAttempt > reconnectInterval)
    {
      lastReconnectAttempt = now;
      Serial.println("Attempting to reconnect...");

      if (connectWSServer())
      {
        Serial.println("Reconnected successfully!");
      }
    }
  }

  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi disconnected, restarting...");
    ESP.restart();
  }

  delay(100);
}