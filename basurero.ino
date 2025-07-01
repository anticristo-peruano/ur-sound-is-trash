/*
🗣️🗣️🗣️ UR SOUND IS TRASH 🗣️🗣️🗣️
(Deberíamos buscar otro nombre.)
*/

// ---------- Librerías ----------
#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>


#include <arduinoFFT.h>
#include "model.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"


// ---------- Pines  ----------
// Sensor de captura de audio ISD1820
#define PIN_AUDIO     12

// Sensor ultrasónico HC-SR04
#define TRIG_PIN      14
#define ECHO_PIN      35

// Sensor de efecto Hall
//#define HALL_PIN      32

// Módulo de relé de 2 canales
#define RELAY1_PIN    33 // Servo
#define RELAY2_PIN    32  // Stepper

// Servomotor MG995
#define SERVO_PIN     27
Servo servo1;

// Motor paso a paso NEMA17 con driver a4988
#define PIN_DIR       25
#define PIN_STEP      26


// ---------- Constantes ---------

// ---------- Parámetros y variables de entorno----------
// Buffers de audio
ArduinoFFT<float> FFT = ArduinoFFT<float>();
#define SAMPLE_RATE        16000            // 16 kHz
#define AUDIO_BUFFER_SIZE  16000             // 0.5 segundos de audio
#define FRAME_LENGTH       400              // 25 ms
#define FRAME_STRIDE       160              // 10 ms
#define FFT_LENGTH         512              // Potencia de 2
#define NUM_FILTERS        41               // Número de filtros Mel
#define NOISE_FLOOR_DB     -35

namespace {
const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *input = |;
TfLiteTensor *output = nullptr;

constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
}

float* window = nullptr;
float** mel_filters = nullptr;
void generate_mel_filterbank(float** mel_filters);

// Funcionamiento HC-SR04
long duration;
int distance;
int maxDistance = 26; // 26 cm máximo umbral de objecto en rango de detección

// Intensidad de campo magnético
const int umbralMaximo = 800; // Ajustar con pruebas en la vida real


// Asumimos que el sistema se puede simplificar a una circunferencia unitaria
const int coordenadas[5][2] = { 
  {0, 1},   // 0 - Bottle, siempre el estado inicial
  {1, 0},   // 1 - Can
  {0, 0},   // 2 - Noise
  {0, -1},  // 3 - Ping pong
  {-1, 0}   // 4 - Paper
};

int estadoActual = 0;
int estadoObjetivo;

// Del NEMA17
const int pasosPorRev = 200;
const int angularMaxima = 800;
const int angularMinima = 3000;

float r1 = 6.37; // mm
float r2 = 40; // mm

// ---------- Programa principal ----------
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

  // ---------- Inicio de constantes y modelo ----------
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

  // ---------- Configuración de pines ----------
  // Configurar pines del ISD1820 para grabación
  pinMode(PIN_AUDIO, INPUT);

  // Configurar pines del sensor ultrasónico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Configurar pines del módulo de relé
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);

  // Estado inicial de los relés (apagados)
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  delay(1000);

  // ---------- Calibración de motores ----------
  // Servomotor
  digitalWrite(RELAY1_PIN, LOW);
  servo1.attach(SERVO_PIN);
  activarServo(true); // Posición inicial
  delay(1000);
  digitalWrite(RELAY1_PIN, HIGH);
  delay(1000);

  // Configurar pines del motor paso a paso (NEMA17 + A4988)
  digitalWrite(RELAY2_PIN, LOW);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  digitalWrite(PIN_DIR, HIGH);  // Dirección hacia adelante
  delay(1000);
  digitalWrite(RELAY2_PIN, HIGH);
  delay(1000);

  // Fin setup
  Serial.println("Preparao.");
}

void loop(){
  // En bucle, va esperando a que un objeto entre en el rango del detector de basura.
  delay(1000);

  bool basura = detectarBasura();
  if (basura) {
    float* audio_buffer = capture_audio();
    capture_audio();
    int frame_count = (AUDIO_BUFFER_SIZE - FRAME_LENGTH) / FRAME_STRIDE + 1;
    float* mfe_output = (float*) malloc(sizeof(float) * frame_count * NUM_FILTERS);
    compute_mfe(audio_buffer, mfe_output, frame_count);

    estadoObjetivo = prediction(mfe_output);
    delay(1000);

    free(audio_buffer);
    free(mfe_output);

    if (estadoObjetivo != 2){ // Solo si no es ruido, que se mueva.
      // Rotación de la plataforma circular
      digitalWrite(RELAY2_PIN, HIGH);
      actuadorStepper(estadoObjetivo);
      delay(100);
      digitalWrite(RELAY2_PIN, LOW);
      delay(100);

      // Apertura y cierre de trampilla
      digitalWrite(RELAY1_PIN, HIGH);
      activarServo(true);
      delay(2000);
      activarServo(false);
      delay(2000);
      digitalWrite(RELAY1_PIN, LOW);
      
      // Actualizar estado actual
      estadoActual = estadoObjetivo;

      /*
      Aunque los dos motores estén en paralelo con la fuente de tensión, cada uno aprovecha el
      total de intensidad de corriente durante su proceso. Ocurre así porque en la programación,
      los dos canales del relé nunca están abiertos al mismo tiempo.

      O es uno, o el otro, o ninguno.
      */
    }
  }
}

// ---------- Funciones agregadas ----------
// Funciones auxiliares de procesamiento de audio}


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



// Función detección de distancia
bool detectarBasura(){
  // Enviar un pulso al TRIG_PIN
  digitalWrite(TRIG_PIN, LOW);  // Asegurarse de que el TRIG_PIN esté en LOW
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); // Enviar un pulso HIGH
  delayMicroseconds(10);        // Duración del pulso
  digitalWrite(TRIG_PIN, LOW);  // Apagar el TRIG_PIN

  // Leer la duración del pulso de vuelta desde el ECHO_PIN
  duration = pulseIn(ECHO_PIN, HIGH);  // Medir el tiempo del pulso
  distance = duration * 0.017;  // La velocidad del sonido es 343 m/s o 0.034 cm/us

  // Si la distancia es menor que un umbral retorna true (objeto detectado)
  if (distance >= 2 && distance <= 20) {
    return true;
  } else {
    return false;
  }
}

// Movimiento stepper
void unPaso(int tiempo) {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(tiempo);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(tiempo);
}

void actuadorStepper (int estadoObjetivo) {
  if (estadoActual != estadoObjetivo) {
    // Parámetros de viaje
    float angulo = calcularRotacion(estadoObjetivo);
    int direccion = (angulo > 0) ? HIGH : LOW;
    float pasos = angulo * (r2 / (1.8 * r1));
    int pasos_enteros = round(pasos);

    // Dirección del motor
    digitalWrite(PIN_DIR, direccion);

    // Pasito a pasito
    for (int i = 0; i < pasos_enteros; i++) {
      unPaso(2500);
    }
  }
}

float calcularRotacion(int estadoObjetivo) {
  int x1 = coordenadas[estadoActual][0];
  int y1 = coordenadas[estadoActual][1];

  int x2 = coordenadas[estadoObjetivo][0];
  int y2 = coordenadas[estadoObjetivo][1];

  // (radianes) * (angulo / radianes)
  float angulo = acos( (x1*x2 + y1*y2) / ( sqrt(pow(x1,2) + pow(y1,2)) * sqrt(pow(x2,2) + pow(y2,2)))) * (180.0 / 3.14159);

  if ((x1*y2 - y1*x2) < 0) { // Producto cruzado. Negativo = antihorario; Positivo = horario.
    angulo *= -1;
  }
  return angulo;
}



// Movimiento servo
void activarServo(bool estado) {
  if (estado) {
    servo1.write(90);  // Mover el servo a 90°, CERRADO
    Serial.println("Servo activado: 180°");
  } else {
    servo1.write(0);  // Mover el servo a 0°, ABIERTO
    Serial.println("Servo desactivado: 0°");
  }
}