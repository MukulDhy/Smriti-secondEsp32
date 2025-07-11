#include "wifi_manager.h"
#include "config.h"
#include "utils.h"

void WiFiManager::connect()
{
    Serial.println("🔄 Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

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
        Utils::restartSystem("WiFi Connection Failed");
    }
}

bool WiFiManager::isConnected()
{
    return WiFi.status() == WL_CONNECTED;
}

int WiFiManager::getSignalStrength()
{
    return WiFi.RSSI();
}