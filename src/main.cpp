/*
 * ESP32 Sensor Node with Network Diagnostics
 * Enhanced with better WiFi handling and Telnet debugging
 */

#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_now.h>
#include <HTTPClient.h>
#include <driver/i2s.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
// ADD these includes at the top with your other includes (after #include <Wire.h>)

// WiFi credentials for backend communication
const char *ssid = "Mukuldhy";
const char *password = "12345678";
const char *serverURL = "http://192.168.1.10:5000/upload-audio";

// Telnet server setup
WiFiServer telnetServer(23);
WiFiClient telnetClient;
bool telnetEnabled = false;

// ESP-NOW Configuration
uint8_t cydMAC[] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF}; // Replace with your CYD ESP32 MAC
esp_now_peer_info_t peerInfo;

// I2S Configuration for INMP441
#define I2S_WS 15
#define I2S_SD 13
#define I2S_SCK 2
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE 16000
#define I2S_SAMPLE_BITS 16
#define I2S_READ_LEN (16 * 1024)
#define RECORD_TIME 5
#define I2S_CHANNEL_NUM 1

// MAX30102 Configuration
MAX30105 particleSensor;
const byte RATE_SIZE = 4;
long rateArray[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
int beatsPerMinute;
bool fingerDetected = false;

// Timing variables
unsigned long lastSensorCheck = 0;
unsigned long lastNetworkCheck = 0;
const unsigned long SENSOR_CHECK_INTERVAL = 10000; // 10 seconds
const unsigned long NETWORK_CHECK_INTERVAL = 5000; // 5 secon

void serialPrintln(String message);
void serialPrint(String message);

// System state
bool micActive = false;
bool hrActive = false;
TaskHandle_t micTaskHandle = NULL;
TaskHandle_t hrTaskHandle = NULL;

// Command structure for ESP-NOW
typedef struct
{
  char command[10];
  int value;
} Command;

// Heart rate data structure
typedef struct
{
  int heartRate;
  bool valid;
} HRData;

// Sensor data structure
typedef struct
{
  int heartRate;
  bool hrValid;
  bool micWorking;
  float audioLevel;
} SensorData;

// Enhanced serial print function for Telnet
// void serialPrint(String message)
// {
//   Serial.print(message); // Always print to USB serial
//   if (telnetEnabled && telnetClient && telnetClient.connected())
//   {
//     telnetClient.print(message); // Also send to Telnet if connected
//   }
// }

// void serialPrintln(String message)
// {
//   Serial.println(message);
//   if (telnetEnabled && telnetClient && telnetClient.connected())
//   {
//     telnetClient.println(message);
//   }
// }

// Network diagnostics function
void printNetworkDiagnostics()
{
  serialPrintln("\n=== Network Diagnostics ===");

  // WiFi Status
  serialPrint("WiFi Status: ");
  switch (WiFi.status())
  {
  case WL_CONNECTED:
    serialPrintln("Connected");
    break;
  case WL_NO_SSID_AVAIL:
    serialPrintln("SSID not available");
    break;
  case WL_CONNECT_FAILED:
    serialPrintln("Connection failed");
    break;
  case WL_CONNECTION_LOST:
    serialPrintln("Connection lost");
    break;
  case WL_DISCONNECTED:
    serialPrintln("Disconnected");
    break;
  default:
    serialPrint("Unknown (");
    serialPrint(String(WiFi.status()));
    serialPrintln(")");
    break;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    serialPrint("SSID: ");
    serialPrintln(WiFi.SSID());
    serialPrint("IP Address: ");
    serialPrintln(WiFi.localIP().toString());
    serialPrint("Gateway: ");
    serialPrintln(WiFi.gatewayIP().toString());
    serialPrint("Subnet Mask: ");
    serialPrintln(WiFi.subnetMask().toString());
    serialPrint("DNS: ");
    serialPrintln(WiFi.dnsIP().toString());
    serialPrint("MAC Address: ");
    serialPrintln(WiFi.macAddress());
    serialPrint("RSSI: ");
    serialPrint(String(WiFi.RSSI()));
    serialPrintln(" dBm");

    // Telnet server status
    serialPrint("Telnet Server: ");
    serialPrintln(telnetEnabled ? "Running on port 23" : "Disabled");

    if (telnetClient && telnetClient.connected())
    {
      serialPrint("Telnet Client: Connected from ");
      serialPrintln(telnetClient.remoteIP().toString());
    }
    else
    {
      serialPrintln("Telnet Client: Not connected");
    }
  }

  serialPrintln("=== End Diagnostics ===\n");
}

// I2S setup
void i2sInit()
{
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = I2S_SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 64,
      .dma_buf_len = 1024,
      .use_apll = 1};

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = -1,
      .data_in_num = I2S_SD};

  i2s_set_pin(I2S_PORT, &pin_config);
  serialPrintln("I2S initialized successfully");
}

// Initialize MAX30102
bool initMAX30102()
{
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    serialPrintln("MAX30102 not found");
    return false;
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  serialPrintln("MAX30102 initialized successfully");
  return true;
}

// Heart rate calculation
int calculateHeartRate()
{
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue))
  {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rateArray[rateSpot++] = beatsPerMinute;
      rateSpot %= RATE_SIZE;

      long total = 0;
      for (byte i = 0; i < RATE_SIZE; i++)
      {
        total += rateArray[i];
      }
      beatsPerMinute = total / RATE_SIZE;
    }
  }

  fingerDetected = (irValue > 50000);
  return fingerDetected ? beatsPerMinute : 0;
}

// Test microphone and get audio level
float testMicrophone()
{
  char *i2s_read_buff = (char *)calloc(1024, sizeof(char));
  size_t bytes_read = 0;
  float audioLevel = 0.0;

  if (i2s_read_buff == NULL)
  {
    serialPrintln("Failed to allocate memory for mic test");
    return 0.0;
  }

  // Read a small sample
  esp_err_t result = i2s_read(I2S_PORT, (void *)i2s_read_buff, 1024, &bytes_read, 100);

  if (result == ESP_OK && bytes_read > 0)
  {
    // Calculate RMS (audio level)
    long sum = 0;
    int16_t *samples = (int16_t *)i2s_read_buff;
    int sampleCount = bytes_read / 2;

    for (int i = 0; i < sampleCount; i++)
    {
      sum += abs(samples[i]);
    }

    audioLevel = (float)sum / sampleCount;
    serialPrint("Mic test - Audio level: ");
    serialPrintln(String(audioLevel));
  }
  else
  {
    serialPrintln("Microphone test failed");
  }

  free(i2s_read_buff);
  return audioLevel;
}

// Check both sensors every 10 seconds
void checkSensors()
{
  serialPrintln("\n=== 10-Second Sensor Check ===");

  // Check Heart Rate Sensor
  serialPrint("Checking MAX30102... ");
  int currentHR = calculateHeartRate();

  if (fingerDetected)
  {
    serialPrint("Heart Rate: ");
    serialPrint(String(currentHR));
    serialPrintln(" BPM - Finger detected");
  }
  else
  {
    serialPrintln("No finger detected on sensor");
  }

  // Check Microphone
  serialPrint("Checking INMP441... ");
  float audioLevel = testMicrophone();

  if (audioLevel > 100) // Threshold for detecting audio
  {
    serialPrintln("Microphone working - Audio detected");
  }
  else if (audioLevel > 10)
  {
    serialPrintln("Microphone working - Low audio level");
  }
  else
  {
    serialPrintln("Microphone - No audio or not working");
  }

  // Send data via ESP-NOW (if initialized)
  SensorData sensorData;
  sensorData.heartRate = currentHR;
  sensorData.hrValid = fingerDetected;
  sensorData.micWorking = (audioLevel > 10);
  sensorData.audioLevel = audioLevel;

  // Uncomment when ESP-NOW is working
  // esp_now_send(cydMAC, (uint8_t *)&sensorData, sizeof(sensorData));

  serialPrintln("=== Check Complete ===\n");
}

// Send heart rate data via ESP-NOW
void sendHeartRateData(int hr, bool valid)
{
  HRData hrData;
  hrData.heartRate = hr;
  hrData.valid = valid;

  esp_err_t result = esp_now_send(cydMAC, (uint8_t *)&hrData, sizeof(hrData));

  if (result == ESP_OK)
  {
    serialPrint("HR sent: ");
    serialPrint(String(hr));
    serialPrint(" BPM, Valid: ");
    serialPrintln(valid ? "Yes" : "No");
  }
  else
  {
    serialPrint("Error sending HR data: ");
    serialPrintln(String(result));
  }
}

// Audio recording and upload task
void microphoneTask(void *pvParameters)
{
  char *i2s_read_buff = (char *)calloc(I2S_READ_LEN, sizeof(char));
  uint8_t *audio_buffer = (uint8_t *)calloc(I2S_SAMPLE_RATE * RECORD_TIME * 2, sizeof(uint8_t));

  while (micActive)
  {
    size_t total_bytes = 0;
    size_t bytes_read = 0;

    serialPrintln("Recording audio...");

    // Record for 5 seconds
    unsigned long startTime = millis();
    while (millis() - startTime < (RECORD_TIME * 1000) && micActive)
    {
      i2s_read(I2S_PORT, (void *)i2s_read_buff, I2S_READ_LEN, &bytes_read, portMAX_DELAY);

      if (total_bytes + bytes_read < I2S_SAMPLE_RATE * RECORD_TIME * 2)
      {
        memcpy(audio_buffer + total_bytes, i2s_read_buff, bytes_read);
        total_bytes += bytes_read;
      }
    }

    if (micActive && total_bytes > 0)
    {
      serialPrint("Recorded ");
      serialPrint(String(total_bytes));
      serialPrintln(" bytes, uploading...");

      // Upload to server
      HTTPClient http;
      http.begin(serverURL);
      http.addHeader("Content-Type", "audio/wav");

      int httpResponseCode = http.POST(audio_buffer, total_bytes);

      if (httpResponseCode > 0)
      {
        String response = http.getString();
        serialPrint("Upload response: ");
        serialPrint(String(httpResponseCode));
        serialPrint(" - ");
        serialPrintln(response);
      }
      else
      {
        serialPrint("Upload failed: ");
        serialPrintln(String(httpResponseCode));
      }

      http.end();
    }
  }

  free(i2s_read_buff);
  free(audio_buffer);
  vTaskDelete(NULL);
}

// Heart rate monitoring task
void heartRateTask(void *pvParameters)
{
  while (hrActive)
  {
    int currentHR = calculateHeartRate();

    if (millis() - lastSensorCheck >= 500)
    {
      sendHeartRateData(currentHR, fingerDetected);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }

  vTaskDelete(NULL);
}

// Start microphone recording
void startMicrophone()
{
  if (!micActive)
  {
    micActive = true;
    xTaskCreate(microphoneTask, "MicTask", 8192, NULL, 1, &micTaskHandle);
    serialPrintln("Microphone started");
  }
}

// Stop microphone recording
void stopMicrophone()
{
  if (micActive)
  {
    micActive = false;
    if (micTaskHandle != NULL)
    {
      vTaskDelete(micTaskHandle);
      micTaskHandle = NULL;
    }
    serialPrintln("Microphone stopped");
  }
}

// Start heart rate monitoring
void startHeartRate()
{
  if (!hrActive)
  {
    hrActive = true;

    // Reset heart rate calculation
    for (byte i = 0; i < RATE_SIZE; i++)
    {
      rateArray[i] = 0;
    }
    rateSpot = 0;
    beatsPerMinute = 0;

    xTaskCreate(heartRateTask, "HRTask", 4096, NULL, 1, &hrTaskHandle);
    serialPrintln("Heart rate monitoring started");
  }
}

// Stop heart rate monitoring
void stopHeartRate()
{
  if (hrActive)
  {
    hrActive = false;
    if (hrTaskHandle != NULL)
    {
      vTaskDelete(hrTaskHandle);
      hrTaskHandle = NULL;
    }
    serialPrintln("Heart rate monitoring stopped");
  }
}

// ESP-NOW receive callback
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len == sizeof(Command))
  {
    Command command;
    memcpy(&command, incomingData, sizeof(command));

    serialPrint("Received command: ");
    serialPrintln(String(command.command));

    if (strcmp(command.command, "MIC_ON") == 0)
    {
      startMicrophone();
    }
    else if (strcmp(command.command, "MIC_OFF") == 0)
    {
      stopMicrophone();
    }
    else if (strcmp(command.command, "HR_ON") == 0)
    {
      startHeartRate();
    }
    else if (strcmp(command.command, "HR_OFF") == 0)
    {
      stopHeartRate();
    }
  }
}

// ESP-NOW send callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Optional: Handle send confirmation
}

// Initialize WiFi for backend communication
void initWiFi()
{
  serialPrint("Connecting to WiFi: ");
  serialPrintln(String(ssid));

  // Set WiFi mode
  WiFi.mode(WIFI_STA);

  // Optional: Set static IP if needed
  // WiFi.config(IPAddress(192,168,1,100), IPAddress(192,168,1,1), IPAddress(255,255,255,0));

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30)
  {
    delay(1000);
    serialPrint(".");
    attempts++;

    // Print status every 5 attempts
    if (attempts % 5 == 0)
    {
      serialPrint(" (Status: ");
      serialPrint(String(WiFi.status()));
      serialPrint(")");
    }
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    serialPrintln("\nWiFi connected successfully!");
    printNetworkDiagnostics();
  }
  else
  {
    serialPrintln("\nWiFi connection failed!");
    serialPrint("Final status: ");
    serialPrintln(String(WiFi.status()));
  }
}

// Initialize ESP-NOW
void initESPNow()
{
  if (esp_now_init() != ESP_OK)
  {
    serialPrintln("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Add peer
  memcpy(peerInfo.peer_addr, cydMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    serialPrintln("Failed to add peer");
    return;
  }

  serialPrintln("ESP-NOW initialized successfully");
  serialPrint("This device MAC: ");
  serialPrintln(WiFi.macAddress());
}

// Setup Telnet server
void setupTelnet()
{
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  telnetEnabled = true;
  serialPrintln("Telnet server started on port 23");
  serialPrint("Telnet address: telnet ");
  serialPrintln(WiFi.localIP().toString());
  serialPrintln("Or use: nc " + WiFi.localIP().toString() + " 23");
}

// Handle Telnet connections
void handleTelnet()
{
  // Check for new clients
  if (telnetServer.hasClient())
  {
    // Disconnect existing client if present
    if (telnetClient && telnetClient.connected())
    {
      serialPrintln("Disconnecting existing Telnet client");
      telnetClient.stop();
    }

    // Accept new client
    telnetClient = telnetServer.available();
    if (telnetClient && telnetClient.connected())
    {
      serialPrint("New Telnet client connected from: ");
      serialPrintln(telnetClient.remoteIP().toString());

      // Send welcome message
      telnetClient.println("=== ESP32 Sensor Node ===");
      telnetClient.println("Telnet connection established");
      telnetClient.println("Type 'help' for commands");
      telnetClient.println("=========================");
    }
  }

  // Handle client disconnection
  if (telnetClient && !telnetClient.connected())
  {
    serialPrintln("Telnet client disconnected");
    telnetClient.stop();
  }

  // Handle incoming Telnet commands
  if (telnetClient && telnetClient.connected() && telnetClient.available())
  {
    String command = telnetClient.readStringUntil('\n');
    command.trim();
    command.toLowerCase();

    serialPrint("Telnet command received: ");
    serialPrintln(command);

    if (command == "help")
    {
      telnetClient.println("Available commands:");
      telnetClient.println("  status - Show system status");
      telnetClient.println("  network - Show network diagnostics");
      telnetClient.println("  sensors - Check sensors now");
      telnetClient.println("  mic_on - Start microphone");
      telnetClient.println("  mic_off - Stop microphone");
      telnetClient.println("  hr_on - Start heart rate monitoring");
      telnetClient.println("  hr_off - Stop heart rate monitoring");
      telnetClient.println("  help - Show this help");
    }
    else if (command == "status")
    {
      telnetClient.println("System Status:");
      telnetClient.println("Microphone: " + String(micActive ? "Active" : "Inactive"));
      telnetClient.println("Heart Rate: " + String(hrActive ? "Active" : "Inactive"));
      telnetClient.println("Uptime: " + String(millis() / 1000) + " seconds");
    }
    else if (command == "network")
    {
      printNetworkDiagnostics();
    }
    else if (command == "sensors")
    {
      checkSensors();
    }
    else if (command == "mic_on")
    {
      startMicrophone();
    }
    else if (command == "mic_off")
    {
      stopMicrophone();
    }
    else if (command == "hr_on")
    {
      startHeartRate();
    }
    else if (command == "hr_off")
    {
      stopHeartRate();
    }
    else
    {
      telnetClient.println("Unknown command: " + command);
      telnetClient.println("Type 'help' for available commands");
    }
  }
}

// ADD these includes at the top with your other includes (after #include <Wire.h>)
#include <WebServer.h>

// ADD these variables after your existing telnet variables
WebServer webServer(80);
String serialLog = "";
const int MAX_LOG_SIZE = 8000;
bool webSerialEnabled = false;

// REPLACE your existing serialPrint and serialPrintln functions with these:
void serialPrint(String message)
{
  Serial.print(message); // Always print to USB serial

  // Add to web log
  serialLog += message;
  if (serialLog.length() > MAX_LOG_SIZE)
  {
    serialLog = serialLog.substring(serialLog.length() - MAX_LOG_SIZE + 1000);
  }

  // Also send to Telnet if connected (keep existing telnet functionality)
  if (telnetEnabled && telnetClient && telnetClient.connected())
  {
    telnetClient.print(message);
  }
}

void serialPrintln(String message)
{
  Serial.println(message);

  // Add to web log with timestamp
  String timestampedMessage = "[" + String(millis() / 1000) + "s] " + message + "\n";
  serialLog += timestampedMessage;
  if (serialLog.length() > MAX_LOG_SIZE)
  {
    serialLog = serialLog.substring(serialLog.length() - MAX_LOG_SIZE + 1000);
  }

  // Also send to Telnet if connected
  if (telnetEnabled && telnetClient && telnetClient.connected())
  {
    telnetClient.println(message);
  }
}

// ADD these new functions BEFORE your setup() function:

void handleRoot()
{
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Sensor Monitor</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { font-family: 'Courier New', monospace; margin: 20px; background: #1a1a1a; color: #00ff00; }
        .container { max-width: 1200px; margin: 0 auto; }
        .header { background: #333; padding: 15px; border-radius: 5px; margin-bottom: 20px; }
        .status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 15px; margin-bottom: 20px; }
        .status-box { background: #2a2a2a; padding: 15px; border-radius: 5px; border-left: 4px solid #00ff00; }
        .log-container { background: #000; padding: 15px; border-radius: 5px; height: 400px; overflow-y: auto; border: 1px solid #333; }
        .controls { margin: 20px 0; }
        .btn { background: #007700; color: white; padding: 8px 16px; border: none; border-radius: 3px; margin: 5px; cursor: pointer; }
        .btn:hover { background: #009900; }
        .btn-danger { background: #770000; }
        .btn-danger:hover { background: #990000; }
        pre { margin: 0; white-space: pre-wrap; font-size: 12px; }
        .online { color: #00ff00; }
        .offline { color: #ff0000; }
        h1, h2 { color: #00ffff; }
    </style>
    <script>
        function sendCommand(cmd) {
            fetch('/command?cmd=' + cmd)
            .then(response => response.text())
            .then(data => {
                console.log('Command sent:', cmd);
                setTimeout(refreshData, 500);
            });
        }
        
        function refreshData() {
            fetch('/status')
            .then(response => response.json())
            .then(data => {
                document.getElementById('uptime').textContent = data.uptime + ' seconds';
                document.getElementById('heap').textContent = data.freeHeap + ' bytes';
                document.getElementById('wifi-status').textContent = data.wifiStatus;
                document.getElementById('wifi-status').className = data.wifiConnected ? 'online' : 'offline';
                document.getElementById('mic-status').textContent = data.micActive ? 'Active' : 'Inactive';
                document.getElementById('mic-status').className = data.micActive ? 'online' : 'offline';
                document.getElementById('hr-status').textContent = data.hrActive ? 'Active' : 'Inactive';
                document.getElementById('hr-status').className = data.hrActive ? 'online' : 'offline';
                document.getElementById('ip').textContent = data.ip;
                document.getElementById('rssi').textContent = data.rssi + ' dBm';
            });
            
            fetch('/log')
            .then(response => response.text())
            .then(data => {
                const logDiv = document.getElementById('log');
                logDiv.innerHTML = '<pre>' + data + '</pre>';
                logDiv.scrollTop = logDiv.scrollHeight;
            });
        }
        
        setInterval(refreshData, 2000);
        window.onload = refreshData;
    </script>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🔧 ESP32 Sensor Node Monitor</h1>
            <p>Wireless Serial Monitor & Control Panel</p>
        </div>
        
        <div class="status-grid">
            <div class="status-box">
                <h3>📊 System Status</h3>
                <p><strong>Uptime:</strong> <span id="uptime">Loading...</span></p>
                <p><strong>Free Heap:</strong> <span id="heap">Loading...</span></p>
            </div>
            
            <div class="status-box">
                <h3>📡 Network</h3>
                <p><strong>WiFi:</strong> <span id="wifi-status">Loading...</span></p>
                <p><strong>IP:</strong> <span id="ip">Loading...</span></p>
                <p><strong>RSSI:</strong> <span id="rssi">Loading...</span></p>
            </div>
            
            <div class="status-box">
                <h3>🎤 Sensors</h3>
                <p><strong>Microphone:</strong> <span id="mic-status">Loading...</span></p>
                <p><strong>Heart Rate:</strong> <span id="hr-status">Loading...</span></p>
            </div>
        </div>
        
        <div class="controls">
            <h3>📋 Controls</h3>
            <button class="btn" onclick="sendCommand('mic_on')">🎤 Start Microphone</button>
            <button class="btn btn-danger" onclick="sendCommand('mic_off')">🛑 Stop Microphone</button>
            <button class="btn" onclick="sendCommand('hr_on')">❤️ Start Heart Rate</button>
            <button class="btn btn-danger" onclick="sendCommand('hr_off')">🛑 Stop Heart Rate</button>
            <button class="btn" onclick="sendCommand('sensors')">🔍 Check Sensors</button>
            <button class="btn" onclick="sendCommand('network')">📡 Network Test</button>
            <button class="btn" onclick="refreshData()">🔄 Refresh</button>
        </div>
        
        <div class="status-box">
            <h3>📜 Live Serial Log</h3>
            <div id="log" class="log-container">
                <pre>Loading serial output...</pre>
            </div>
        </div>
    </div>
</body>
</html>
)rawliteral";

  webServer.send(200, "text/html", html);
}

void handleStatus()
{
  String json = "{";
  json += "\"uptime\":" + String(millis() / 1000) + ",";
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"wifiStatus\":\"" + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected") + "\",";
  json += "\"wifiConnected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  json += "\"micActive\":" + String(micActive ? "true" : "false") + ",";
  json += "\"hrActive\":" + String(hrActive ? "true" : "false") + ",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"rssi\":" + String(WiFi.RSSI());
  json += "}";

  webServer.send(200, "application/json", json);
}

void handleLog()
{
  webServer.send(200, "text/plain", serialLog);
}

void handleCommand()
{
  String cmd = webServer.arg("cmd");
  cmd.toLowerCase();

  serialPrintln("Web command received: " + cmd);

  if (cmd == "mic_on")
  {
    startMicrophone();
  }
  else if (cmd == "mic_off")
  {
    stopMicrophone();
  }
  else if (cmd == "hr_on")
  {
    startHeartRate();
  }
  else if (cmd == "hr_off")
  {
    stopHeartRate();
  }
  else if (cmd == "sensors")
  {
    checkSensors();
  }
  else if (cmd == "network")
  {
    printNetworkDiagnostics();
  }

  webServer.send(200, "text/plain", "Command executed: " + cmd);
}

void setupWebSerial()
{
  webServer.on("/", handleRoot);
  webServer.on("/status", handleStatus);
  webServer.on("/log", handleLog);
  webServer.on("/command", handleCommand);

  webServer.begin();
  webSerialEnabled = true;

  serialPrintln("Web Serial Monitor started!");
  serialPrintln("Open your browser and go to: http://" + WiFi.localIP().toString());
  serialPrintln("This is much more reliable than Telnet!");
}

void handleWebSerial()
{
  if (webSerialEnabled)
  {
    webServer.handleClient();
  }
}

// ADD this line in your setup() function right after setupTelnet():
// setupWebSerial();

// ADD this line in your loop() function right after handleTelnet():
// handleWebSerial();

void setup()
{
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize

  serialPrintln("\n=== ESP32 Sensor Node Starting ===");
  serialPrint("Chip Model: ");
  serialPrintln(ESP.getChipModel());
  serialPrint("Free Heap: ");
  serialPrintln(String(ESP.getFreeHeap()));

  // Initialize WiFi first
  initWiFi();

  // Setup Telnet only if WiFi is connected
  if (WiFi.status() == WL_CONNECTED)
  {
    setupTelnet();
  }
  else
  {
    serialPrintln("Skipping Telnet setup - WiFi not connected");
  }

  // Initialize I2C for MAX30102
  Wire.begin();

  // Initialize sensors
  i2sInit();

  if (!initMAX30102())
  {
    serialPrintln("MAX30102 initialization failed!");
    // Don't halt, continue with microphone only
  }

  // Initialize ESP-NOW (commented out for now to avoid brownout)
  // initESPNow();

  serialPrintln("=== Setup Complete ===");
  serialPrintln("Sensors will be checked every 10 seconds...");
  serialPrintln("Network diagnostics every 5 seconds...");

  lastSensorCheck = millis();
  lastNetworkCheck = millis();

  // Print initial diagnostics
  if (WiFi.status() == WL_CONNECTED)
  {
    printNetworkDiagnostics();
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    setupWebSerial();
  }
}

void loop()
{
  // Handle Telnet connections and commands
  if (telnetEnabled)
  {
    handleTelnet();
  }

  // Check sensors every 10 seconds
  if (millis() - lastSensorCheck >= SENSOR_CHECK_INTERVAL)
  {
    checkSensors();
    lastSensorCheck = millis();
  }

  // Network diagnostics every 5 seconds
  if (millis() - lastNetworkCheck >= NETWORK_CHECK_INTERVAL)
  {
    // Check WiFi connection and reconnect if needed
    if (WiFi.status() != WL_CONNECTED)
    {
      serialPrintln("WiFi disconnected, attempting reconnection...");
      WiFi.reconnect();
      delay(1000);
    }

    lastNetworkCheck = millis();
  }
  // Handle Web Serial Monitor
  handleWebSerial();

  delay(100); // Small delay to prevent watchdog issues
}