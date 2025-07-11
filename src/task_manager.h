#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <Arduino.h>

class TaskManager
{
private:
    static TaskHandle_t audioTaskHandle;
    static TaskHandle_t heartRateTaskHandle;
    static TaskHandle_t gyroscopeTaskHandle;
    static TaskHandle_t sensorTaskHandle;
    static unsigned long lastSensorRead;

    static void audioTask(void *parameter);
    static void heartRateTask(void *parameter);
    static void gyroscopeTask(void *parameter);
    static void sensorTask(void *parameter);

public:
    static void startAllTasks();
    static void stopAllTasks();
    static void startAudioTask();
    static void stopAudioTask();
    static void startHeartRateTask();
    static void stopHeartRateTask();
    static void startGyroscopeTask();
    static void stopGyroscopeTask();
};

#endif