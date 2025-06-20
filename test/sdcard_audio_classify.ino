// Leer un archivo WAV de una microSD y clasificar con TFLite Micro

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "model_data.h"
#include <arduinoFFT.h>

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#define SAMPLE_RATE 16000
#define AUDIO_BUFFER_SIZE 16000
#define FRAME_LENGTH 400
#define FRAME_STRIDE 160
#define FFT_LENGTH 512
#define NUM_FILTERS 41
#define NOISE_FLOOR_DB -57.0

#define SD_CS_PIN 5  // Cambia según tu conexión física

float audio_buffer[AUDIO_BUFFER_SIZE];
float mfe_output[4018];
float window[FRAME_LENGTH];
float mel_filters[NUM_FILTERS][FFT_LENGTH / 2 + 1];
arduinoFFT FFT = arduinoFFT();

constexpr int kTensorArenaSize = 50 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
tflite::MicroInterpreter* interpreter;
TfLiteTensor* input;
TfLiteTensor* output;

float hz_to_mel(float hz) { return 2595.0f * log10(1.0f + hz / 700.0f); }
float mel_to_hz(float mel) { return 700.0f * (pow(10.0f, mel / 2595.0f) - 1.0f); }

void generate_hamming_window() {
  for (int i = 0; i < FRAME_LENGTH; i++)
    window[i] = 0.54 - 0.46 * cos(2 * PI * i / (FRAME_LENGTH - 1));
}

void generate_mel_filterbank() {
  float low_mel = hz_to_mel(80);
  float high_mel = hz_to_mel(SAMPLE_RATE / 2);
  float mel_points[NUM_FILTERS + 2];
  int bin[NUM_FILTERS + 2];

  for (int i = 0; i < NUM_FILTERS + 2; i++)
    mel_points[i] = mel_to_hz(low_mel + (high_mel - low_mel) * i / (NUM_FILTERS + 1));
  for (int i = 0; i < NUM_FILTERS + 2; i++)
    bin[i] = floor((FFT_LENGTH + 1) * mel_points[i] / SAMPLE_RATE);

  for (int i = 0; i < NUM_FILTERS; i++) {
    for (int k = 0; k < FFT_LENGTH / 2 + 1; k++) {
      float w = 0.0;
      if (k >= bin[i] && k <= bin[i + 1])
        w = (float)(k - bin[i]) / (bin[i + 1] - bin[i]);
      else if (k >= bin[i + 1] && k <= bin[i + 2])
        w = (float)(bin[i + 2] - k) / (bin[i + 2] - bin[i + 1]);
      mel_filters[i][k] = w;
    }
  }
}

bool load_wav(const char* path) {
  File f = SD.open(path);
  if (!f) {
    Serial.println("No se pudo abrir el archivo WAV");
    return false;
  }

  // Saltar encabezado WAV (44 bytes)
  for (int i = 0; i < 44; i++) f.read();

  int i = 0;
  while (f.available() && i < AUDIO_BUFFER_SIZE) {
    int16_t sample = f.read() | (f.read() << 8); // Little-endian
    audio_buffer[i++] = sample / 32768.0f;
  }
  f.close();
  return i == AUDIO_BUFFER_SIZE;
}

void compute_mfe(float* audio) {
  int frame_count = (AUDIO_BUFFER_SIZE - FRAME_LENGTH) / FRAME_STRIDE + 1;
  for (int f = 0; f < frame_count; f++) {
    float frame[FRAME_LENGTH];
    float vReal[FFT_LENGTH];
    float vImag[FFT_LENGTH];
    for (int i = 0; i < FRAME_LENGTH; i++) {
      frame[i] = audio[f * FRAME_STRIDE + i] * window[i];
    }
    for (int i = 0; i < FRAME_LENGTH; i++) vReal[i] = frame[i];
    for (int i = FRAME_LENGTH; i < FFT_LENGTH; i++) vReal[i] = 0;
    for (int i = 0; i < FFT_LENGTH; i++) vImag[i] = 0;
    FFT.Windowing(vReal, FFT_LENGTH, FFT_WIN_TYP_RECTANGLE, FFT_FORWARD);
    FFT.Compute(vReal, vImag, FFT_LENGTH, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, FFT_LENGTH);
    for (int m = 0; m < NUM_FILTERS; m++) {
      float energy = 0.0;
      for (int k = 0; k < FFT_LENGTH / 2 + 1; k++) {
        energy += vReal[k] * vReal[k] * mel_filters[m][k];
      }
      energy = 10.0 * log10(max(energy, 1e-30f));
      energy = (energy - NOISE_FLOOR_DB) / ((-1 * NOISE_FLOOR_DB) + 12);
      energy = constrain(energy, 0.0, 1.0);
      mfe_output[f * NUM_FILTERS + m] = energy;
    }
  }
}

int run_inference(float* features, float* result_probs, int max_classes) {
  for (int i = 0; i < 4018; i++) {
    input->data.f[i] = features[i];
  }
  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("Inference failed");
    return -1;
  }
  for (int i = 0; i < max_classes; i++) {
    result_probs[i] = output->data.f[i];
  }
  return 0;
}

int get_top_class(const float* probs, int num_classes) {
  int top_index = 0;
  float top_prob = probs[0];
  for (int i = 1; i < num_classes; i++) {
    if (probs[i] > top_prob) {
      top_prob = probs[i];
      top_index = i;
    }
  }
  return top_index;
}

void setup() {
  Serial.begin(115200);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Error inicializando SD");
    while (true);
  }

  generate_hamming_window();
  generate_mel_filterbank();

  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::AllOpsResolver resolver;
  const tflite::Model* model = tflite::GetModel(model_quantized_int8_tflite);
  interpreter = new tflite::MicroInterpreter(model, resolver, tensor_arena, kTensorArenaSize, &micro_error_reporter);
  interpreter->AllocateTensors();
  input = interpreter->input(0);
  output = interpreter->output(0);

  if (load_wav("/audio.wav")) {
    compute_mfe(audio_buffer);
    float probs[5];
    if (run_inference(mfe_output, probs, 5) == 0) {
      int predicted = get_top_class(probs, 5);
      Serial.print("Clase predicha: ");
      Serial.println(predicted);
      for (int i = 0; i < 5; i++) {
        Serial.print("Clase "); Serial.print(i);
        Serial.print(": "); Serial.println(probs[i], 4);
      }
    }
  }
}

void loop() {
  // Nada en el loop: ejecución única al arrancar
}
