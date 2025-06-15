#ifndef AUDIO_SENSOR_H
#define AUDIO_SENSOR_H

#include <Arduino.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include "../config.h"

struct AudioData
{
    bool isValid;
    float amplitude;
    float dominantFrequency;
    float noiseLevel;
    String rawSamples;
    unsigned long timestamp;
};

class AudioSensor
{
public:
    AudioSensor() : initialized(false), active(true)
    {
        audioBuffer = new int32_t[BUFFER_SIZE];
        vReal = new double[FFT_SIZE];
        vImag = new double[FFT_SIZE];
        FFT = ArduinoFFT<double>(vReal, vImag, FFT_SIZE, SAMPLE_RATE);
    }

    ~AudioSensor()
    {
        if (audioBuffer)
            delete[] audioBuffer;
        if (vReal)
            delete[] vReal;
        if (vImag)
            delete[] vImag;
        if (initialized)
            i2s_driver_uninstall(I2S_NUM_0);
    }

    bool begin()
    {
        if (initialized)
            return true;

        // Configure I2S
        i2s_config_t i2s_config = {
            .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
            .sample_rate = SAMPLE_RATE,
            .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
            .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
            .communication_format = I2S_COMM_FORMAT_STAND_I2S,
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
            .dma_buf_count = 4,
            .dma_buf_len = BUFFER_SIZE,
            .use_apll = false,
            .tx_desc_auto_clear = false,
            .fixed_mclk = 0};

        i2s_pin_config_t pin_config = {
            .bck_io_num = I2S_SCK_PIN,
            .ws_io_num = I2S_WS_PIN,
            .data_out_num = I2S_PIN_NO_CHANGE,
            .data_in_num = I2S_SD_PIN};

        esp_err_t result = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
        if (result != ESP_OK)
        {
            Serial.println("Failed to install I2S driver");
            return false;
        }

        result = i2s_set_pin(I2S_NUM_0, &pin_config);
        if (result != ESP_OK)
        {
            Serial.println("Failed to set I2S pins");
            return false;
        }

        initialized = true;
        Serial.println("Audio sensor initialized");
        return true;
    }

    AudioData readData()
    {
        AudioData data = {false, 0, 0, 0, "", millis()};

        if (!initialized || !active)
            return data;

        size_t bytes_read = 0;
        esp_err_t result = i2s_read(I2S_NUM_0, audioBuffer, BUFFER_SIZE * sizeof(int32_t), &bytes_read, portMAX_DELAY);

        if (result == ESP_OK && bytes_read > 0)
        {
            size_t samples_read = bytes_read / sizeof(int32_t);

            data.amplitude = calculateAmplitude(audioBuffer, samples_read);
            data.dominantFrequency = findDominantFrequency();
            data.noiseLevel = data.amplitude;
            data.rawSamples = encodeAudioSamples(audioBuffer, min(samples_read, (size_t)MAX_AUDIO_SAMPLES));
            data.isValid = true;
        }

        return data;
    }

    bool isConnected() const { return initialized; }
    bool isActive() const { return active; }

    void setActive(bool state)
    {
        active = state;
        if (initialized)
        {
            if (active)
                i2s_start(I2S_NUM_0);
            else
                i2s_stop(I2S_NUM_0);
        }
    }

    void calibrate()
    {
        if (!initialized)
            return;

        Serial.println("Calibrating audio sensor...");
        float noiseFloor = 0;
        for (int i = 0; i < 10; i++)
        {
            AudioData data = readData();
            noiseFloor += data.amplitude;
            delay(100);
        }
        noiseFloor /= 10.0;
        Serial.println("Audio calibration complete. Noise floor: " + String(noiseFloor));
    }

private:
    i2s_config_t i2s_config;
    i2s_pin_config_t pin_config;
    int32_t *audioBuffer;
    double *vReal;
    double *vImag;
    ArduinoFFT<double> FFT;
    bool initialized;
    bool active;

    float calculateAmplitude(int32_t *buffer, size_t samples)
    {
        float sum = 0;
        for (size_t i = 0; i < samples; i++)
        {
            float sample = (float)buffer[i] / INT32_MAX;
            sum += sample * sample;
        }
        return sqrt(sum / samples);
    }

    float findDominantFrequency()
    {
        for (int i = 0; i < FFT_SIZE; i++)
        {
            vReal[i] = (i < BUFFER_SIZE) ? (double)audioBuffer[i] / INT32_MAX : 0.0;
            vImag[i] = 0.0;
        }

        FFT.windowing(vReal, FFT_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(vReal, vImag, FFT_SIZE, FFT_FORWARD);
        FFT.complexToMagnitude(vReal, vImag, FFT_SIZE);

        return FFT.majorPeak(vReal, FFT_SIZE, SAMPLE_RATE);
    }

    String encodeAudioSamples(int32_t *buffer, size_t samples)
    {
        String encoded = "";
        for (size_t i = 0; i < samples; i += 4)
        {
            encoded += String(buffer[i] >> 16, HEX);
            if (i + 4 < samples)
                encoded += ",";
        }
        return encoded;
    }
};

#endif