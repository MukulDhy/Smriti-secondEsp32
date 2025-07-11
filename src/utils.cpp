#include "utils.h"
#include "websocket_manager.h"
#include <ArduinoJson.h>

void Utils::restartSystem(const char *reason)
{
    Serial.printf("🔄 RESTARTING SYSTEM: %s\n", reason);

    if (WebSocketManager::isConnected())
    {
        JsonDocument doc;
        doc["type"] = "system-restart";
        doc["reason"] = reason;
        doc["timestamp"] = millis();

        String message;
        serializeJson(doc, message);

        // Try to send restart notification
        // Note: We can't access client directly, so we'd need to add a method to WebSocketManager
        delay(1000);
    }

    ESP.restart();
}

void Utils::printSystemInfo()
{
    Serial.println("=== System Information ===");
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("Uptime: %lu ms\n", millis());
    Serial.printf("Chip Model: %s\n", ESP.getChipModel());
    Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
    Serial.printf("Flash Size: %d bytes\n", ESP.getFlashChipSize());
    Serial.println("========================");
}

unsigned long Utils::getUptime()
{
    return millis();
}

size_t Utils::getFreeHeap()
{
    return ESP.getFreeHeap();
}