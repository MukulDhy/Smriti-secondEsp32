#include "task_manager.h"
#include "config.h"
#include "websocket_manager.h"
#include "audio_manager.h"
#include "utils.h"

// Static member definitions
TaskHandle_t TaskManager::micTaskHandle = nullptr;
TaskHandle_t TaskManager::sensorTaskHandle = nullptr;
unsigned long TaskManager::lastSensorRead = 0;

void TaskManager::micTask(void *parameter)
{
    Serial.println("🎤 Starting microphone task...");

    size_t bytesIn = 0;
    int consecutiveErrors = 0;
    const int maxConsecutiveErrors = 50;

    while (true)
    {
        if (WebSocketManager::isConnected() && AudioManager::isStreaming() && WebSocketManager::isEstablished())
        {
            esp_err_t result = i2s_read(I2S_PORT, AudioManager::getBuffer(), AudioManager::getBufferSize(), &bytesIn, 100);

            if (result == ESP_OK && bytesIn > 0)
            {
                WebSocketManager::sendBinary((const char *)AudioManager::getBuffer(), bytesIn);
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
                    Utils::restartSystem("I2S Read Errors");
                }
            }
        }
        else
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
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

void TaskManager::stopAllTasks()
{
    if (micTaskHandle != nullptr)
    {
        vTaskDelete(micTaskHandle);
        micTaskHandle = nullptr;
    }

    if (sensorTaskHandle != nullptr)
    {
        vTaskDelete(sensorTaskHandle);
        sensorTaskHandle = nullptr;
    }
}