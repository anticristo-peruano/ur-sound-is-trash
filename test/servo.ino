#include <ESP32Servo.h>

Servo mg996r;
int servoPin = 18;

void setup() {
  Serial.begin(115200);
  mg996r.attach(servoPin, 1000, 2000);  // Mejor rango para MG996R
  Serial.println("Iniciando...");
}

void loop() {
  Serial.println("Moviendo a 0°");
  mg996r.write(0);
  delay(2000);

  Serial.println("Moviendo a 90°");
  mg996r.write(90);
  delay(2000);

  Serial.println("Moviendo a 180°");
  mg996r.write(180);
  delay(2000);
}