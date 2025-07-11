#include "sensor_manager.h"
#include "config.h"
#include "websocket_manager.h"
#include <ArduinoJson.h>

// Include sensor libraries

// Static member definitions
bool SensorManager::audioActive = false;
int16_t SensorManager::audioBuffer[1024];

bool SensorManager::heartRateActive = false;
MAX30105 *SensorManager::heartRateSensor = nullptr;
TwoWire *SensorManager::heartRateWire = nullptr;
float SensorManager::heartRate = 0.0;
float SensorManager::spO2 = 0.0;
unsigned long SensorManager::lastHeartRateRead = 0;

bool SensorManager::gyroscopeActive = false;
MPU6050 *SensorManager::gyroscopeSensor = nullptr;
TwoWire *SensorManager::gyroscopeWire = nullptr;
float SensorManager::accelX = 0.0, SensorManager::accelY = 0.0, SensorManager::accelZ = 0.0;
float SensorManager::gyroX = 0.0, SensorManager::gyroY = 0.0, SensorManager::gyroZ = 0.0;
float SensorManager::temperature = 0.0;
unsigned long SensorManager::lastGyroscopeRead = 0;

bool SensorManager::sensorStatus[SENSOR_COUNT] = {false, false, false};

void SensorManager::init()
{
    Serial.println("🔧 Initializing sensor manager...");

    // Initialize all sensors but don't start them
    initAudio();
    initHeartRate();
    initGyroscope();

    Serial.println("✓ Sensor manager initialized");
}

bool SensorManager::initAudio()
{
    Serial.println("🎤 Initializing audio sensor (INMP441)...");

    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = AUDIO_SAMPLE_RATE,
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
        return false;
    }

    result = i2s_set_pin(I2S_PORT, &pin_config);
    if (result != ESP_OK)
    {
        Serial.printf("❌ I2S pin config failed: %d\n", result);
        return false;
    }

    result = i2s_set_clk(I2S_PORT, AUDIO_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    if (result != ESP_OK)
    {
        Serial.printf("❌ I2S clock config failed: %d\n", result);
        return false;
    }

    Serial.println("✓ Audio sensor initialized");
    return true;
}

bool SensorManager::initHeartRate()
{
    Serial.println("❤️  Initializing heart rate sensor (MAX30102)...");

    // Initialize I2C for heart rate sensor
    heartRateWire = new TwoWire(0);
    heartRateWire->begin(HR_SDA, HR_SCL, HR_I2C_FREQ);

    // Initialize MAX30102
    heartRateSensor = new MAX30105();
    if (!heartRateSensor->begin(*heartRateWire))
    {
        Serial.println("❌ MAX30102 not found");
        return false;
    }

    // Configure sensor
    heartRateSensor->setup();
    heartRateSensor->setPulseAmplitudeRed(0x0A);
    heartRateSensor->setPulseAmplitudeGreen(0);

    Serial.println("✓ Heart rate sensor initialized");
    return true;
}

bool SensorManager::initGyroscope()
{
    Serial.println("🔄 Initializing gyroscope sensor (GY-87)...");

    // Initialize I2C for gyroscope sensor
    gyroscopeWire = new TwoWire(1);
    gyroscopeWire->begin(GYRO_SDA, GYRO_SCL, GYRO_I2C_FREQ);

    // Initialize MPU6050
    gyroscopeSensor = new MPU6050();
    gyroscopeSensor->initialize();

    if (!gyroscopeSensor->testConnection())
    {
        Serial.println("❌ MPU6050 connection failed");
        return false;
    }

    Serial.println("✓ Gyroscope sensor initialized");
    return true;
}

void SensorManager::startSensor(SensorType sensor)
{
    switch (sensor)
    {
    case SENSOR_AUDIO:
        if (!audioActive)
        {
            esp_err_t result = i2s_start(I2S_PORT);
            if (result == ESP_OK)
            {
                audioActive = true;
                sensorStatus[SENSOR_AUDIO] = true;
                Serial.println("🎤 Audio streaming started");
                WebSocketManager::sendStatusUpdate();
            }
            else
            {
                Serial.printf("❌ Failed to start audio: %d\n", result);
            }
        }
        break;

    case SENSOR_HEART_RATE:
        if (!heartRateActive && heartRateSensor != nullptr)
        {
            heartRateActive = true;
            sensorStatus[SENSOR_HEART_RATE] = true;
            lastHeartRateRead = millis();
            Serial.println("❤️  Heart rate monitoring started");
            WebSocketManager::sendStatusUpdate();
        }
        break;

    case SENSOR_GYROSCOPE:
        if (!gyroscopeActive && gyroscopeSensor != nullptr)
        {
            gyroscopeActive = true;
            sensorStatus[SENSOR_GYROSCOPE] = true;
            lastGyroscopeRead = millis();
            Serial.println("🔄 Gyroscope monitoring started");
            WebSocketManager::sendStatusUpdate();
        }
        break;
    }
}

void SensorManager::stopSensor(SensorType sensor)
{
    switch (sensor)
    {
    case SENSOR_AUDIO:
        if (audioActive)
        {
            i2s_stop(I2S_PORT);
            audioActive = false;
            sensorStatus[SENSOR_AUDIO] = false;
            Serial.println("🎤 Audio streaming stopped");
            WebSocketManager::sendStatusUpdate();
        }
        break;

    case SENSOR_HEART_RATE:
        if (heartRateActive)
        {
            heartRateActive = false;
            sensorStatus[SENSOR_HEART_RATE] = false;
            Serial.println("❤️  Heart rate monitoring stopped");
            WebSocketManager::sendStatusUpdate();
        }
        break;

    case SENSOR_GYROSCOPE:
        if (gyroscopeActive)
        {
            gyroscopeActive = false;
            sensorStatus[SENSOR_GYROSCOPE] = false;
            Serial.println("🔄 Gyroscope monitoring stopped");
            WebSocketManager::sendStatusUpdate();
        }
        break;
    }
}

bool SensorManager::isSensorActive(SensorType sensor)
{
    if (sensor >= 0 && sensor < SENSOR_COUNT)
    {
        return sensorStatus[sensor];
    }
    return false;
}

void SensorManager::stopAllSensors()
{
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        stopSensor(static_cast<SensorType>(i));
    }
}

// Audio methods
bool SensorManager::isAudioActive() { return audioActive; }
int16_t *SensorManager::getAudioBuffer() { return audioBuffer; }
size_t SensorManager::getAudioBufferSize() { return sizeof(audioBuffer); }

bool SensorManager::readAudioData(size_t *bytesRead)
{
    if (!audioActive)
        return false;

    esp_err_t result = i2s_read(I2S_PORT, audioBuffer, sizeof(audioBuffer), bytesRead, 100);
    return (result == ESP_OK && *bytesRead > 0);
}

// Heart Rate methods
bool SensorManager::isHeartRateActive() { return heartRateActive; }

bool SensorManager::readHeartRateData()
{
    if (!heartRateActive || heartRateSensor == nullptr)
        return false;

    unsigned long now = millis();
    if (now - lastHeartRateRead < HEART_RATE_INTERVAL)
        return false;

    lastHeartRateRead = now;

    if (heartRateSensor->available())
    {
        uint32_t irValue = heartRateSensor->getIR();

        if (irValue > 50000)
        {
            // Simple heart rate calculation (simplified)
            // In a real implementation, you'd use a proper algorithm
            heartRate = 75.0 + (irValue % 1000) / 40.0; // Simulated for demo
            spO2 = 95.0 + (irValue % 500) / 100.0;      // Simulated for demo
            return true;
        }
    }

    return false;
}

float SensorManager::getHeartRate() { return heartRate; }
float SensorManager::getSpO2() { return spO2; }

// Gyroscope methods
bool SensorManager::isGyroscopeActive() { return gyroscopeActive; }

bool SensorManager::readGyroscopeData()
{
    if (!gyroscopeActive || gyroscopeSensor == nullptr)
        return false;

    unsigned long now = millis();
    if (now - lastGyroscopeRead < GYROSCOPE_INTERVAL)
        return false;

    lastGyroscopeRead = now;

    int16_t ax, ay, az, gx, gy, gz;
    gyroscopeSensor->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert to proper units
    accelX = ax / 16384.0; // ±2g
    accelY = ay / 16384.0;
    accelZ = az / 16384.0;

    gyroX = gx / 131.0; // ±250°/s
    gyroY = gy / 131.0;
    gyroZ = gz / 131.0;

    temperature = gyroscopeSensor->getTemperature() / 340.0 + 36.53;

    return true;
}

void SensorManager::getAcceleration(float *x, float *y, float *z)
{
    *x = accelX;
    *y = accelY;
    *z = accelZ;
}

void SensorManager::getRotation(float *x, float *y, float *z)
{
    *x = gyroX;
    *y = gyroY;
    *z = gyroZ;
}

float SensorManager::getTemperature() { return temperature; }

void SensorManager::printSensorStatus()
{
    Serial.println("=== Sensor Status ===");
    Serial.printf("Audio: %s\n", audioActive ? "ACTIVE" : "INACTIVE");
    Serial.printf("Heart Rate: %s\n", heartRateActive ? "ACTIVE" : "INACTIVE");
    Serial.printf("Gyroscope: %s\n", gyroscopeActive ? "ACTIVE" : "INACTIVE");
    Serial.println("====================");
}

String SensorManager::getSensorStatusJson()
{
    JsonDocument doc;
    doc["audio"] = audioActive;
    doc["heartRate"] = heartRateActive;
    doc["gyroscope"] = gyroscopeActive;

    String result;
    serializeJson(doc, result);
    return result;
}