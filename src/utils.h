#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

class Utils
{
public:
    static void restartSystem(const char *reason);
    static void printSystemInfo();
    static unsigned long getUptime();
    static size_t getFreeHeap();
};

#endif