#include "task_manager.h"
#include "config.h"
#include "websocket_manager.h"
#include "sensor_manager.h"
#include "utils.h"
#include <ArduinoJson.h>

// Static member definitions
TaskHandle_t TaskManager::audioTaskHandle = nullptr;
TaskHandle_t TaskManager::heartRateTaskHandle = nullptr;
TaskHandle_t TaskManager::gyroscopeTaskHandle = nullptr;
TaskHandle_t TaskManager::sensorTaskHandle = nullptr;
unsigned long TaskManager::lastSensorRead = 0;

void TaskManager::audioTask(void *parameter)
{
    Serial.println("🎤 Starting audio task...");

    size_t bytesRead = 0;
    int consecutiveErrors = 0;
    const int maxConsecutiveErrors = 50;

    while (true)
    {
        if (WebSocketManager::isConnected() && SensorManager::isAudioActive() && WebSocketManager::isEstablished())
        {
            if (SensorManager::readAudioData(&bytesRead))
            {
                // Send audio data as binary
                WebSocketManager::sendBinary((const char *)SensorManager::getAudioBuffer(), bytesRead);
                consecutiveErrors = 0;

                // Optional: Send a text message indicating audio data was sent
                JsonDocument doc;
                doc["type"] = "audio-data";
                doc["timestamp"] = millis();
                doc["dataSize"] = bytesRead;

                String message;
                serializeJson(doc, message);
                WebSocketManager::sendText(message);
            }
            else
            {
                consecutiveErrors++;
                if (consecutiveErrors % 20 == 0)
                {
                    Serial.printf("⚠️ Audio read error, consecutive errors: %d\n", consecutiveErrors);
                }

                if (consecutiveErrors > maxConsecutiveErrors)
                {
                    Serial.println("❌ Too many audio errors - restarting");
                    Utils::restartSystem("Audio Read Errors");
                }
            }

            // Small delay to prevent overwhelming the WebSocket
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        else
        {
            // If not active, wait longer
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void TaskManager::heartRateTask(void *parameter)
{
    Serial.println("❤️  Starting heart rate task...");

    while (true)
    {
        if (WebSocketManager::isConnected() && SensorManager::isHeartRateActive() && WebSocketManager::isEstablished())
        {
            if (SensorManager::readHeartRateData())
            {
                JsonDocument doc;
                doc["type"] = "sensor-data";
                doc["sensorType"] = "heartRate";
                doc["timestamp"] = millis();

                JsonObject data = doc["data"].to<JsonObject>();
                data["heartRate"] = SensorManager::getHeartRate();
                data["spO2"] = SensorManager::getSpO2();

                String message;
                serializeJson(doc, message);
                WebSocketManager::sendText(message);
            }

            vTaskDelay(HEART_RATE_INTERVAL / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void TaskManager::gyroscopeTask(void *parameter)
{
    Serial.println("🔄 Starting gyroscope task...");

    while (true)
    {
        if (WebSocketManager::isConnected() && SensorManager::isGyroscopeActive() && WebSocketManager::isEstablished())
        {
            if (SensorManager::readGyroscopeData())
            {
                JsonDocument doc;
                doc["type"] = "sensor-data";
                doc["sensorType"] = "gyroscope";
                doc["timestamp"] = millis();

                JsonObject data = doc["data"].to<JsonObject>();

                float ax, ay, az, gx, gy, gz;
                SensorManager::getAcceleration(&ax, &ay, &az);
                SensorManager::getRotation(&gx, &gy, &gz);

                JsonObject accel = data["acceleration"].to<JsonObject>();
                accel["x"] = ax;
                accel["y"] = ay;
                accel["z"] = az;

                JsonObject gyro = data["gyroscope"].to<JsonObject>();
                gyro["x"] = gx;
                gyro["y"] = gy;
                gyro["z"] = gz;

                data["temperature"] = SensorManager::getTemperature();

                String message;
                serializeJson(doc, message);
                WebSocketManager::sendText(message);
            }

            vTaskDelay(GYROSCOPE_INTERVAL / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void TaskManager::sensorTask(void *parameter)
{
    Serial.println("📊 Starting sensor task...");

    while (true)
    {
        unsigned long now = millis();

        if (now - lastSensorRead >= SENSOR_READ_INTERVAL && WebSocketManager::isEstablished())
        {
            lastSensorRead = now;
            WebSocketManager::sendSensorData();
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void TaskManager::startAllTasks()
{
    // Start sensor task on core 0 (for periodic system updates)
    xTaskCreatePinnedToCore(
        sensorTask,
        "sensorTask",
        8000,
        NULL,
        1,
        &sensorTaskHandle,
        0);

    Serial.println("✓ Background sensor task started");
}

void TaskManager::stopAllTasks()
{
    if (audioTaskHandle != nullptr)
    {
        vTaskDelete(audioTaskHandle);
        audioTaskHandle = nullptr;
    }

    if (heartRateTaskHandle != nullptr)
    {
        vTaskDelete(heartRateTaskHandle);
        heartRateTaskHandle = nullptr;
    }

    if (gyroscopeTaskHandle != nullptr)
    {
        vTaskDelete(gyroscopeTaskHandle);
        gyroscopeTaskHandle = nullptr;
    }

    if (sensorTaskHandle != nullptr)
    {
        vTaskDelete(sensorTaskHandle);
        sensorTaskHandle = nullptr;
    }
}

void TaskManager::startAudioTask()
{
    if (audioTaskHandle == nullptr)
    {
        xTaskCreatePinnedToCore(
            audioTask,
            "audioTask",
            10000,
            NULL,
            2,
            &audioTaskHandle,
            1);
        Serial.println("✓ Audio task started");
    }
    else
    {
        Serial.println("⚠️ Audio task already running");
    }
}

void TaskManager::stopAudioTask()
{
    if (audioTaskHandle != nullptr)
    {
        vTaskDelete(audioTaskHandle);
        audioTaskHandle = nullptr;
        Serial.println("✓ Audio task stopped");
    }
}

void TaskManager::startHeartRateTask()
{
    if (heartRateTaskHandle == nullptr)
    {
        xTaskCreatePinnedToCore(
            heartRateTask,
            "heartRateTask",
            8000,
            NULL,
            2,
            &heartRateTaskHandle,
            1);
        Serial.println("✓ Heart rate task started");
    }
    else
    {
        Serial.println("⚠️ Heart rate task already running");
    }
}

void TaskManager::stopHeartRateTask()
{
    if (heartRateTaskHandle != nullptr)
    {
        vTaskDelete(heartRateTaskHandle);
        heartRateTaskHandle = nullptr;
        Serial.println("✓ Heart rate task stopped");
    }
}

void TaskManager::startGyroscopeTask()
{
    if (gyroscopeTaskHandle == nullptr)
    {
        xTaskCreatePinnedToCore(
            gyroscopeTask,
            "gyroscopeTask",
            8000,
            NULL,
            2,
            &gyroscopeTaskHandle,
            1);
        Serial.println("✓ Gyroscope task started");
    }
    else
    {
        Serial.println("⚠️ Gyroscope task already running");
    }
}

void TaskManager::stopGyroscopeTask()
{
    if (gyroscopeTaskHandle != nullptr)
    {
        vTaskDelete(gyroscopeTaskHandle);
        gyroscopeTaskHandle = nullptr;
        Serial.println("✓ Gyroscope task stopped");
    }
}