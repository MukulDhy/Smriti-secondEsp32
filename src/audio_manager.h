#ifndef AUDIO_MANAGER_H
#define AUDIO_MANAGER_H

#include <Arduino.h>
#include <driver/i2s.h>

class AudioManager
{
private:
    static bool isAudioStreaming;
    static int16_t sBuffer[];

public:
    static void init();
    static void startStreaming();
    static void stopStreaming();
    static bool isStreaming();
    static int16_t *getBuffer();
    static size_t getBufferSize();
};

#endif