#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "driver/i2s.h"

// --- Pin Definitions ---
// SD Card (Native SPI)
#define SD_SCK 4
#define SD_MISO 5
#define SD_MOSI 6
#define SD_CS 7

// PCM1808 (I2S)
#define I2S_MCLK_IO GPIO_NUM_2
#define I2S_BCK_IO GPIO_NUM_0
#define I2S_WS_IO GPIO_NUM_1
#define I2S_DIN_IO GPIO_NUM_3

// --- Recording Settings ---
#define SAMPLE_RATE 16000                                                // 16kHz is very stable for SD card writing
#define RECORD_TIME 30                                                   // Record duration in seconds
#define CHANNELS 1                                                       // Mono recording
#define BIT_DEPTH 16                                                     // Save as 16-bit WAV to save SD bandwidth
#define AUDIO_THRESHOLD 5000                                             // Minimum amplitude to consider as "audio" (adjust based on your mic's sensitivity)
#define THRESHOLD_DURATION 2000                                          // Time in ms that the audio must be below the threshold to start recording
#define THRESHOLD_SAMPLE_COUNT (SAMPLE_RATE * THRESHOLD_DURATION / 1000) // Number of samples corresponding to the threshold duration

// --- General Config ---
#define GRAPH_WIDTH 100
#define GRAPH_MAX_VAL (10000) // Max value for scaling the graph (adjust as needed)
#define LOG_INTERVAL_MS 100

File audioFile;

int fileIndex = 1;

// Function to write the 44-byte WAV header
void writeWavHeader(File file, uint32_t sampleRate, uint16_t bitsPerSample, uint16_t channels, uint32_t dataSize)
{
  byte header[44];
  uint32_t byteRate = sampleRate * channels * (bitsPerSample / 8);
  uint32_t totalSize = dataSize + 36;
  uint16_t blockAlign = channels * (bitsPerSample / 8);

  memcpy(&header[0], "RIFF", 4);
  memcpy(&header[4], &totalSize, 4);
  memcpy(&header[8], "WAVEfmt ", 8);
  uint32_t fmtSize = 16;
  memcpy(&header[16], &fmtSize, 4);
  uint16_t format = 1; // PCM
  memcpy(&header[20], &format, 2);
  memcpy(&header[22], &channels, 2);
  memcpy(&header[24], &sampleRate, 4);
  memcpy(&header[28], &byteRate, 4);
  memcpy(&header[32], &blockAlign, 2);
  memcpy(&header[34], &bitsPerSample, 2);
  memcpy(&header[36], "data", 4);
  memcpy(&header[40], &dataSize, 4);

  file.seek(0);
  file.write(header, 44);
}

void setup_i2s()
{
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // Matches your RIN wiring
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 512,
      .use_apll = false};

  i2s_pin_config_t pin_config = {
      .mck_io_num = I2S_MCLK_IO,
      .bck_io_num = I2S_BCK_IO,
      .ws_io_num = I2S_WS_IO,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_DIN_IO};

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

void recordAudio()
{

  // 1. Prepare the WAV File
  audioFile = SD.open("/rec_" + String(fileIndex) + ".wav", FILE_WRITE);
  if (!audioFile)
  {
    Serial.println("Failed to create file on SD!");
    while (1)
      ; // Halt
  }

  // Write a blank header to hold the space (we'll overwrite it at the end)
  byte emptyHeader[44] = {0};
  audioFile.write(emptyHeader, 44);

  // 2. Start Recording Loop
  Serial.println(">>> RECORDING STARTED (5 Seconds) <<<");
  Serial.println("Speak into the microphone...");

  uint32_t samples_to_record = SAMPLE_RATE * RECORD_TIME;
  uint32_t samples_written = 0;
  int32_t i2s_raw_buffer[256]; // Buffer to hold incoming 32-bit data
  int16_t pcm_16_buffer[256];  // Buffer to hold converted 16-bit data

  while (samples_written < samples_to_record)
  {
    size_t bytes_read = 0;

    // Read a chunk of data from the I2S microphone
    esp_err_t result = i2s_read(I2S_NUM_0, i2s_raw_buffer, sizeof(i2s_raw_buffer), &bytes_read, portMAX_DELAY);

    if (result == ESP_OK && bytes_read > 0)
    {
      int num_samples = bytes_read / sizeof(int32_t);

      // Convert 32-bit I2S data to 16-bit PCM for the WAV file
      for (int i = 0; i < num_samples; i++)
      {
        // Shift the 24-bit PCM1808 data down to 16-bit space
        pcm_16_buffer[i] = (int16_t)(i2s_raw_buffer[i] >> 14);
      }

      // Write the 16-bit chunk directly to the SD card
      audioFile.write((const uint8_t *)pcm_16_buffer, num_samples * sizeof(int16_t));
      samples_written += num_samples;
    }
  }

  // 5. Finalize the File
  Serial.println(">>> RECORDING STOPPED <<<");

  // Overwrite the placeholder header with the actual file sizes
  writeWavHeader(audioFile, SAMPLE_RATE, BIT_DEPTH, CHANNELS, samples_written * sizeof(int16_t));
  audioFile.close();

  Serial.println("Saved as 'rec_01.wav'. You can now unplug the SD card and play it on your computer!");
  fileIndex++;
}

void awaitThreshold(bool lessThan = false)
{
  int32_t i2s_raw_buffer[256];
  uint64_t mean_amplitude = 0; // 64‑bit to avoid overflow when multiplying
  uint64_t sample_count = 0;   // also 64‑bit for the same reason

  unsigned long lastLog = millis();

  while (true)
  {
    size_t bytes_read = 0;
    esp_err_t result = i2s_read(I2S_NUM_0, i2s_raw_buffer, sizeof(i2s_raw_buffer), &bytes_read, portMAX_DELAY);
    if (result == ESP_OK && bytes_read > 0)
    {
      int num_samples = bytes_read / sizeof(int32_t);
      uint64_t samplegroup_amplitude = 0;
      for (int i = 0; i < num_samples; i++)
      {
        // shift the 24‑bit PCM data down to 16‑bit space, like in recordAudio
        int32_t scaled = i2s_raw_buffer[i] >> 14;
        samplegroup_amplitude += abs(scaled);
      }
      // compute new mean using 64‑bit intermediates to avoid overflow
      uint64_t totalCount = sample_count + (uint64_t)num_samples;
      mean_amplitude = (mean_amplitude * sample_count + samplegroup_amplitude) / totalCount;
      sample_count = totalCount;

      unsigned long now = millis();
      if (now - lastLog >= LOG_INTERVAL_MS)
      {
        lastLog = now;
        // raw value plus bar graph
        Serial.print("Mean amp: ");
        Serial.print(mean_amplitude);

        // scale to GRAPH_WIDTH, using twice the threshold as a rough maximum
        int barLen;
        if (mean_amplitude >= GRAPH_MAX_VAL)
        {
          barLen = GRAPH_WIDTH;
        }
        else
        {
          // safe 64-bit scaling
          barLen = (int)((mean_amplitude * (uint64_t)GRAPH_WIDTH) / GRAPH_MAX_VAL);
        }
        Serial.print(" [");
        for (int j = 0; j < barLen; j++)
          Serial.print('#');
        for (int j = barLen; j < GRAPH_WIDTH; j++)
          Serial.print(' ');
        Serial.println("]");
      }

      if (sample_count > THRESHOLD_SAMPLE_COUNT)
      {
        Serial.print("Current mean amplitude: ");
        Serial.println(mean_amplitude);
        if ((lessThan ? mean_amplitude < AUDIO_THRESHOLD : mean_amplitude > AUDIO_THRESHOLD))
        {
          Serial.println("Threshold detected. Ready!");
          return;
        }
        sample_count = 0;   // Reset count to avoid overflow and keep the mean responsive to recent changes
        mean_amplitude = 0; // Reset mean for the next batch of samples
      }
    }
  }
}
void setup()
{
  Serial.begin(115200);
  delay(3000); // Wait for power to stabilize

  Serial.println("\n--- I2S to SD WAV Recorder ---");

  // 1. Initialize SD Card
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  // Use 10MHz SPI speed for stable audio writing
  if (!SD.begin(SD_CS, SPI, 10000000))
  {
    Serial.println("SD Card Mount Failed! Check connections.");
    while (1)
      ; // Halt
  }
  Serial.println("SD Card Mounted successfully.");

  // 2. Initialize I2S
  setup_i2s();
  Serial.println("I2S Mic Initialized.");

  // 3. Get a list of existing files to determine the next filename
  while (SD.exists(String("/rec_") + String(fileIndex) + String(".wav")))
  {
    fileIndex++;
  }

  // Wait for the volume to drop below the threshold, logging the current level
  Serial.println("Waiting for silence to start recording...");
  awaitThreshold(true);
}

void loop()
{
  // Wait for the volume to rise above the threshold, logging the current level
  Serial.println("Waiting for sound to start recording...");
  awaitThreshold(false);

  // Record audio until the volume drops below the threshold again
  recordAudio();
}