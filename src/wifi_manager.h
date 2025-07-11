#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>

class WiFiManager
{
public:
    static void connect();
    static bool isConnected();
    static int getSignalStrength();
};

#endif