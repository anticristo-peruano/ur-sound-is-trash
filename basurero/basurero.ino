/*
🗣️🗣️🗣️ UR SOUND IS TRASH 🗣️🗣️🗣️
(Deberíamos buscar otro nombre.)
*/

// ---------- Librerías ----------
#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <arduinoFFT.h>
#include "model_data.h"

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

// ---------- Pines  ----------
// Sensor de captura de audio ISD1820
#define PIN_AUDIO     34

// Sensor ultrasónico HC-SR04
#define TRIG_PIN      13
#define ECHO_PIN      27

// Sensor de efecto Hall
#define HALL_PIN      32

// Módulo de relé de 2 canales
#define RELAY1_PIN    2 // Servo
#define RELAY2_PIN    4  // Stepper

// Servomotor MG995
#define SERVO_PIN     5
Servo servo1;

// Motor paso a paso NEMA17 con driver a4988
#define PIN_DIR       18
#define PIN_STEP      19


// ---------- Constantes ---------

// ---------- Parámetros y variables de entorno----------
// Buffers de audio
arduinoFFT FFT = arduinoFFT();
#define SAMPLE_RATE 16000
#define AUDIO_BUFFER_SIZE 16000
#define FRAME_LENGTH 400
#define FRAME_STRIDE 160
#define FFT_LENGTH 512
#define NUM_FILTERS 41
#define NOISE_FLOOR_DB -57.0

float audio_buffer[AUDIO_BUFFER_SIZE];
float mfe_output[4018];
float window[FRAME_LENGTH];
float mel_filters[NUM_FILTERS][FFT_LENGTH / 2 + 1];

// Modelo de aprendizaje profundo
constexpr int kTensorArenaSize = 50 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
tflite::MicroInterpreter* interpreter;
TfLiteTensor* input;
TfLiteTensor* output;

// Funcionamiento HC-SR04
long duration;
int distance;

// Intensidad de campo magnético
const int umbralMaximo = 800; // Ajustar con pruebas en la vida real

// Distancia de detección (en cm)
int maxDistancia = 3;

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


// ---------- Programa principal ----------
void setup() {
  Serial.begin(115200);  // Comunicación serial
  analogReadResolution(12);

  // ---------- Inicio de constantes y modelo ----------
  generate_hamming_window();
  generate_mel_filterbank();

  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::AllOpsResolver resolver;
  const tflite::Model* model = tflite::GetModel(model_data);
  interpreter = new tflite::MicroInterpreter(model, resolver, tensor_arena, kTensorArenaSize, &micro_error_reporter);
  interpreter->AllocateTensors();
  input = interpreter->input(0);
  output = interpreter->output(0);

  // ---------- Configuración de pines ----------
  // Configurar pines del ISD1820 para grabación
  pinMode(PIN_AUDIO, INPUT);

  // Configurar pines del sensor ultrasónico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Configurar pines del sensor de efecto Hall
  pinMode(HALL_PIN, INPUT);

  // Configurar pines del módulo de relé
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);

  // Estado inicial de los relés (apagados)
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  delay(1000);

  // ---------- Calibración de motores ----------
  // Servomotor
  digitalWrite(RELAY1_PIN, HIGH);
  servo1.attach(SERVO_PIN);
  activarServo(true); // Posición inicial
  delay(1000);
  digitalWrite(RELAY1_PIN, LOW);
  delay(1000);

  // Configurar pines del motor paso a paso (NEMA17 + A4988)
  digitalWrite(RELAY2_PIN, HIGH);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  digitalWrite(PIN_DIR, HIGH);  // Dirección hacia adelante
  calibrarMotor();
  delay(1000);
  digitalWrite(RELAY2_PIN, LOW);
  delay(1000);

  // Fin setup
  Serial.println("Preparao.");
}

void loop(){
  // En bucle, va esperando a que un objeto entre en el rango del detector de basura.
  delay(1000);

  bool basura = detectarBasura();
  if (basura) {
    capture_audio();
    compute_mfe(audio_buffer);
    estadoObjetivo = prediction(mfe_output);
    delay(1000);

    if (estadoObjetivo != 2){ // Solo si no es ruido, que se mueva.
      // Rotación de la plataforma circular
      digitalWrite(RELAY2_PIN, HIGH);
      actuadorStepper(estadoObjetivo);
      delay(100);
      digitalWrite(RELAY2_PIN, LOW);
      delay(100);

      // Apertura y cierre de trampilla
      digitalWrite(RELAY1_PIN, HIGH);
      activarServo(false);
      delay(2000);
      activarServo(true);
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
// Funciones auxiliares de procesamiento de audio
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

void generate_mel_filterbank() {
  float low_mel = hz_to_mel(80);
  float high_mel = hz_to_mel(SAMPLE_RATE / 2);
  float mel_points[NUM_FILTERS + 2];

  for (int i = 0; i < NUM_FILTERS + 2; i++) {
    mel_points[i] = mel_to_hz(low_mel + (high_mel - low_mel) * i / (NUM_FILTERS + 1));
  }

  int bin[NUM_FILTERS + 2];
  for (int i = 0; i < NUM_FILTERS + 2; i++) {
    bin[i] = floor((FFT_LENGTH + 1) * mel_points[i] / SAMPLE_RATE);
  }

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

// Función de captura de audio
void capture_audio() {
  unsigned long start_time = micros();
  for (int i = 0; i < AUDIO_BUFFER_SIZE; i++) {
    int raw = analogRead(PIN_AUDIO);
    float centered = (raw - 2048.0f) / 2048.0f;
    audio_buffer[i] = centered;
    while ((micros() - start_time) < ((i + 1) * 1000000L / SAMPLE_RATE));
  }
}

// Funciones de clasificación
int prediction(float* features) {
  for (int i = 0; i < 4018; i++) {
    input->data.f[i] = features[i];
  }
  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("Inference failed");
    return -1;
  }
  int top_index = 0;
  float top_prob = output->data.f[0];
  for (int i = 1; i < output->dims->data[1]; i++) {
    if (output->data.f[i] > top_prob) {
      top_prob = output->data.f[i];
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
  distance = duration * 0.034 / 2;  // La velocidad del sonido es 343 m/s o 0.034 cm/us

  // Si la distancia es menor que un umbral retorna true (objeto detectado)
  if (distance <= maxDistancia) {
    return true;
  } else {
    return false;
  }
}

// Movimiento stepper
void unPaso (tiempo) {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(tiempo);  // Ajusta la velocidad de los pasos
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(tiempo);
}

void actuadorStepper (int estadoObjetivo) {
  if (estadoActual != estadoObjetivo) {
    // Parámetros de viaje
    float angulo = calcularRotacion(estadoObjetivo);
    int direccion = (angulo > 0) ? HIGH : LOW;
    int pasos = abs(angulo) * pasosPorRev / 360.0;

    // Dirección del motor
    digitalWrite(PIN_DIR, direccion);

    // Pasito a pasito
    for (int i = 0; i < pasos; i++){
      unPaso(angularMaxima);
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

void calibrarMotor () {
  int intensidadCampo = 0;
  int velocidad = angularMaxima;

  // El motor empieza a girar indefinidamente
  while (true) {
    // Leer el valor del sensor de Hall (intensidad del campo magnético)
    intensidadCampo = analogRead(HALL_PIN);
    unPaso(velocidad);

    if (intensidadCampo < umbralMaximo) {
      // A medida que nos acercamos, reducimos la velocidad progresivamente
      // Cuanto más alta es la señal, más lenta se vuelve la rotación
      velocidad = map(intensidadCampo, 0, umbralMaximo, angularMinima, angularMaxima);
    }else{
      // Si la intensidad del campo alcanza el umbral máximo, el motor se detiene porque llego a (0,1)
      break;
    }
  }
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