#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <Arduino.h>

class TaskManager
{
private:
    static TaskHandle_t micTaskHandle;
    static TaskHandle_t sensorTaskHandle;
    static unsigned long lastSensorRead;

    static void micTask(void *parameter);
    static void sensorTask(void *parameter);

public:
    static void startAllTasks();
    static void stopAllTasks();
};

#endif