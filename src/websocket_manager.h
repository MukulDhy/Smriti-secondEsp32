#ifndef WEBSOCKET_MANAGER_H
#define WEBSOCKET_MANAGER_H

#include <Arduino.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>

using namespace websockets;

class WebSocketManager
{
private:
    static WebsocketsClient client;
    static bool isWebSocketConnected;
    static bool connectionEstablished;
    static bool deviceInfoSent;
    static unsigned long lastStatusSend;
    static unsigned long lastPingReceived;
    static unsigned long lastPingSent;
    static unsigned long lastReconnectAttempt;
    static int reconnectAttempts;

    static void onEventsCallback(WebsocketsEvent event, String data);
    static void onMessageCallback(WebsocketsMessage message);
    static void sendDeviceInfo();
    static void sendPing();

public:
    static bool connect();
    static bool isConnected();
    static bool isEstablished();
    static void poll();
    static void handlePeriodicUpdates(unsigned long now);
    static void handleReconnection(unsigned long now);
    static void sendStatusUpdate();
    static void sendSensorData();
    static void sendBinary(const char *data, size_t length);
};

#endif