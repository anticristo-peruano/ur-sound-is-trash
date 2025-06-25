#include <arduinoFFT.h>
#include "model.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

// ========================== CONFIGURACIÓN ============================= //
#define SAMPLE_RATE        16000            // 16 kHz
#define AUDIO_BUFFER_SIZE  16000             // 0.5 segundos de audio
#define FRAME_LENGTH       400              // 25 ms
#define FRAME_STRIDE       160              // 10 ms
#define FFT_LENGTH         512              // Potencia de 2
#define NUM_FILTERS        41               // Número de filtros Mel
#define NOISE_FLOOR_DB     -35
#define PIN_AUDIO          34               // GPIO34 como entrada analógica en ESP32

// ========================== MODELO ============================= //

namespace {
const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *input = nullptr;
TfLiteTensor *output = nullptr;

constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
}

// ========================== VARIABLES GLOBALES ======================= //
ArduinoFFT<float> FFT = ArduinoFFT<float>();

float* window = nullptr;
float** mel_filters = nullptr;
void generate_mel_filterbank(float** mel_filters);

const int coordenadas[5][2] = { 
  {0, 1},   // 0 - Bottle, siempre el estado inicial
  {1, 0},   // 1 - Can
  {0, 0},   // 2 - Noise
  {0, -1},  // 3 - Ping pong
  {-1, 0}   // 4 - Paper
};

// ========================== SETUP ============================= //
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  delay(1000);
  pinMode(PIN_AUDIO, INPUT);

  window = (float*) malloc(sizeof(float) * FRAME_LENGTH);
  mel_filters = (float**) malloc(sizeof(float*) * NUM_FILTERS);
  for (int i = 0; i < NUM_FILTERS; i++) {
    mel_filters[i] = (float*) malloc(sizeof(float) * (FFT_LENGTH / 2 + 1));
  }

  generate_hamming_window();
  generate_mel_filterbank(mel_filters);

  model = tflite::GetModel(model_float32_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf(
      "Model provided is schema version %d not equal to supported "
      "version %d.",
      model->version(), TFLITE_SCHEMA_VERSION
    );
    return;
  }

  static tflite::MicroMutableOpResolver<9> resolver;
  resolver.AddFullyConnected();
  resolver.AddReshape();
  resolver.AddSoftmax();  
  resolver.AddShape();
  resolver.AddStridedSlice();
  resolver.AddPack();
  resolver.AddExpandDims();
  resolver.AddConv2D();
  resolver.AddMaxPool2D();

  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed");
    return;
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

}

void loop() {
  Serial.println("Capturando 1 segundo de audio...");
  float* audio_buffer = capture_audio();
  
  Serial.println("Audio capturado. Mostrando 10 muestras:");
  for (int i = 0; i < 10; i++) {
    Serial.print(audio_buffer[i]);
    Serial.print(", ");
  }
  Serial.println();
  Serial.println("Últimas 10 muestras:");
  for (int i = AUDIO_BUFFER_SIZE - 10; i < AUDIO_BUFFER_SIZE; i++) {
    Serial.print(audio_buffer[i], 0);
    Serial.print(", ");
  }
  Serial.println();

  int frame_count = (AUDIO_BUFFER_SIZE - FRAME_LENGTH) / FRAME_STRIDE + 1;
  float* mfe_output = (float*) malloc(sizeof(float) * frame_count * NUM_FILTERS);
  compute_mfe(audio_buffer, mfe_output, frame_count);

  Serial.println("Características MFE:");
  for (int i = 0; i < frame_count * NUM_FILTERS; i++) {
    Serial.print(mfe_output[i], 6);
    Serial.print(", ");
  }
  Serial.println("\n");

  Serial.println(prediction(mfe_output));
  free(audio_buffer);
  free(mfe_output);

  delay(200);
}

// ================= FUNCIONES AUXILIARES =================== //

float hz_to_mel(float hz) {
  return 2595.0f * log10(1.0f + hz / 700.0f);
}

float mel_to_hz(float mel) {
  return 700.0f * (pow(10.0f, mel / 2595.0f) - 1.0f);
}

void generate_hamming_window() {
  for (int i = 0; i < FRAME_LENGTH; i++) {
    window[i] = 0.54 - 0.46 * cos(2 * PI * i / (FRAME_LENGTH - 1));
  }
}

void generate_mel_filterbank(float** mel_filters) {
  for (int i = 0; i < NUM_FILTERS; i++) {
    for (int j = 0; j <= FFT_LENGTH / 2; j++) {
      mel_filters[i][j] = 0.0f;
    }
  }

  float low_mel = hz_to_mel(80);
  float high_mel = hz_to_mel(SAMPLE_RATE / 2);
  float mel_points[NUM_FILTERS + 2];

  for (int i = 0; i < NUM_FILTERS + 2; i++) {
    mel_points[i] = low_mel + (high_mel - low_mel) * i / (NUM_FILTERS + 1);
  }

  float hz_points[NUM_FILTERS + 2];
  int bin[NUM_FILTERS + 2];

  for (int i = 0; i < NUM_FILTERS + 2; i++) {
    hz_points[i] = mel_to_hz(mel_points[i]);
    bin[i] = (int) floor((FFT_LENGTH + 1) * hz_points[i] / SAMPLE_RATE);

    if (bin[i] < 0) bin[i] = 0;
    if (bin[i] > FFT_LENGTH / 2) bin[i] = FFT_LENGTH / 2;
  }

  for (int i = 0; i < NUM_FILTERS; i++) {
    for (int j = bin[i]; j < bin[i + 1]; j++) {
      if (j >= 0 && j <= FFT_LENGTH / 2 && bin[i + 1] != bin[i]) {
        mel_filters[i][j] = (float)(j - bin[i]) / (bin[i + 1] - bin[i]);
      }
    }

    for (int j = bin[i + 1]; j < bin[i + 2]; j++) {
      if (j >= 0 && j <= FFT_LENGTH / 2 && bin[i + 2] != bin[i + 1]) {
        mel_filters[i][j] = (float)(bin[i + 2] - j) / (bin[i + 2] - bin[i + 1]);
      }
    }
  }
}


void compute_mfe(float* audio, float* mfe_output, int frame_count) {
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

    FFT.compute(vReal, vImag, FFT_LENGTH, FFT_FORWARD);
    FFT.complexToMagnitude(vReal, vImag, FFT_LENGTH);

    for (int m = 0; m < NUM_FILTERS; m++) {
      float energy = 0.0;
      for (int k = 0; k < FFT_LENGTH / 2 + 1; k++) {
        energy += vReal[k] * vReal[k] * mel_filters[m][k];
      }

      // Replicando comportamiento de Edge Impulse
      energy = 10.0 * log10f(max(energy,1e-30f));
      energy = (energy - NOISE_FLOOR_DB) / ((-1.0 * NOISE_FLOOR_DB) + 12.0) - 2.0;
      energy = constrain(energy, 0.0, 1.0);
      //energy = round(energy * 256.0);
      //energy = constrain(energy,0.0f,255.0f);
      //energy = energy / 256.0f;

      mfe_output[f * NUM_FILTERS + m] = energy;
    }
  }
}

float* capture_audio() {
  float* output = (float*) malloc(sizeof(float) * AUDIO_BUFFER_SIZE);
  unsigned long start_time = micros();
  for (int i = 0; i < AUDIO_BUFFER_SIZE; i++) {
    output[i] = analogRead(PIN_AUDIO) - 2048.0f + 150.0f;
    while ((micros() - start_time) < ((i + 1) * 1000000L / SAMPLE_RATE));
  }
  return output;
}

int prediction(float* features) {
  for (int i = 0; i < 4018; i++) {
    input->data.f[i] = features[i];  // sin cuantizar
  }

  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    Serial.println("Error invoking.");
    return -1;
  }

  int top_index = 0;
  float top_score = output->data.f[0];

  for (int i = 0; i < output->dims->data[1]; i++) {
    Serial.print("Clase ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(output->data.f[i], 6);  // más precisión

    if (output->data.f[i] > top_score) {
      top_score = output->data.f[i];
      top_index = i;
    }
  }
  return top_index;
}