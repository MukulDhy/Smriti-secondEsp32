#include "audio_manager.h"
#include "config.h"
#include "websocket_manager.h"

// Static member definitions
bool AudioManager::isAudioStreaming = false;
int16_t AudioManager::sBuffer[BUFFER_LEN];

void AudioManager::init()
{
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = BUFFER_COUNT,
      .dma_buf_len = BUFFER_LEN,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0};

  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = -1,
      .data_in_num = I2S_SD};

  esp_err_t result = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (result != ESP_OK)
  {
    Serial.printf("❌ I2S install failed: %d\n", result);
    return;
  }

  result = i2s_set_pin(I2S_PORT, &pin_config);
  if (result != ESP_OK)
  {
    Serial.printf("❌ I2S pin config failed: %d\n", result);
    return;
  }

  result = i2s_set_clk(I2S_PORT, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  if (result != ESP_OK)
  {
    Serial.printf("❌ I2S clock config failed: %d\n", result);
    return;
  }

  result = i2s_start(I2S_PORT);
  if (result != ESP_OK)
  {
    Serial.printf("❌ I2S start failed: %d\n", result);
    return;
  }

  Serial.println("✓ I2S initialized successfully");
}

void AudioManager::startStreaming()
{
  if (!isAudioStreaming)
  {
    isAudioStreaming = true;
    Serial.println("🎤 Audio streaming started");
    WebSocketManager::sendStatusUpdate();
  }
}

void AudioManager::stopStreaming()
{
  if (isAudioStreaming)
  {
    isAudioStreaming = false;
    Serial.println("🎤 Audio streaming stopped");
    WebSocketManager::sendStatusUpdate();
  }
}

bool AudioManager::isStreaming()
{
  return isAudioStreaming;
}

int16_t* AudioManager::getBuffer()
{
  return sBuffer;
}

size_t AudioManager::getBufferSize()
{
  return BUFFER_LEN * sizeof(int16_t);
}