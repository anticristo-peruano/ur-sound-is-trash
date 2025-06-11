/*
🗣️🗣️🗣️ UR SOUND IS TRASH 🗣️🗣️🗣️
(Deberíamos buscar otro nombre.)
*/

// ---------- Librerías ----------
#include <Servo.h>
#include <WiFi.h>
#include <math.h>


// ---------- Pines  ----------
// Sensor de captura de audio ISD1820
#define PIN_REC       26
#define PIN_AUDIO     34
uint16_t audioBuffer[1024];

// Sensor ultrasónico HC-SR04
#define TRIG_PIN      13
#define ECHO_PIN      27
long duration;
int distance;

// Sensor de efecto Hall
#define HALL_PIN      34

// Módulo de relé de 2 canales
#define RELAY1_PIN    12 // Servo
#define RELAY2_PIN    2  // Stepper

// Servomotor MG995
#define SERVO_PIN     32
Servo servo1;

// Motor paso a paso NEMA17 con driver a4988
#define PIN_DIR       33
#define PIN_STEP      32


// ---------- Parámetros y variables de entorno ----------
// Configuración de WiFi (No públicos lol)
const char* ssid = "";
const char* password = "";
const char* host = "";
const uint16_t port = 5000;
WiFiClient client;

// Intensidad de campo magnético
const int umbralMaximo = 800; // Ajustar con pruebas en la vida real

// Distancia de detección (en cm)
int maxDistancia = 3;

// Asumimos que el sistema se puede simplificar a una circunferencia unitaria
const int coordenadas[4][2] = { 
  {0, 1},   // Bottle, siempre el estado inicial
  {1, 0},   // Can
  {0, -1},  // Noise + Pong
  {-1, 0}   // Paper
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
  connect();

  // ---------- Configuración de pines ----------
  // Configurar pines del ISD1820 para grabación
  pinMode(PIN_REC, OUTPUT);
  pinMode(PIN_AUDIO, INPUT);
  digitalWrite(PIN_REC, HIGH); // Mantenerlo en HIGH para permitir la grabación
  
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

void loop() {
  // En bucle, va esperando a que un objeto entre en el rango del detector de basura.
  delay(1000);

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    delay(5000);
  }else{
    bool basura = detectarBasura();
    if (basura) {
      // Apenas la detecta, abre el micrófono y empieza a grabar.
      uint16_t audioData = captureAudio();

      // Manda el audio al modelo de predicción por internet y recibe la clase como entero posicional
      estadoObjetivo = sendToModel(audioData);
      delay(1000);

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

// Captura de audio
uint16_t* captureAudio() {
  // Iniciar grabación
  digitalWrite(PIN_REC, LOW);  // Iniciar grabación (baja a LOW)
  delay(10000);  // Grabar durante 10 segundos
  digitalWrite(PIN_REC, HIGH); // Detener grabación (sube a HIGH)

  // Capturar las muestras de audio
  uint16_t audioData[1024];  // Buffer para las muestras de audio

  for (int i = 0; i < 1024; i++) {
    audioBuffer[i] = analogRead(PIN_AUDIO);  // Leer el valor de la señal
    delayMicroseconds(100);  // Ajusta la velocidad de captura (aproximadamente 10kHz)
  }

  return audioBuffer;
}

// Funciones de WiFI
void connect(){
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
  }
  Serial.println("Conectado a WiFi.")
}

int sendToModel(uint16_t* audioData){
  int response = 0;

  if (client.connect(host, port)) {
    Serial.println("Conexión establecida con el servidor.");

    // Crear una cadena CSV con los valores de audio almacenados en audioBuffer
    String audioCSV = "";
    for (int i = 0; i < 1024; i++) {
      audioCSV += String(audioData[i]);
      if (i < 1023) audioCSV += ",";  // Separar por comas
    }

    // Enviar la cadena CSV al servidor
    client.print(audioCSV); 

    // Esperar respuesta del servidor
    String responseStr = client.readString();
    response = responseStr.toInt();
  } else {
    Serial.println("Error al conectar al servidor.");
  }

  // Cerrar la conexión
  client.stop();
  return response;
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
