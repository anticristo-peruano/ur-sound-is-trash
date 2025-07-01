// Nuestro relé funciona a VCC --> 5V
// Las compuertas se activan con 1 lógico a 5V

#define RELAY1 33  // IN1
#define RELAY2 32  // IN2

void setup() {
  Serial.begin(115200);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);

  digitalWrite(RELAY1, HIGH); // COM to NC
  digitalWrite(RELAY2, HIGH); // COM to NC

  Serial.println("Pruebas.");

  delay(1000);
  Serial.println("Activando relé 1 (compuerta 1)...");
  digitalWrite(RELAY1, LOW);  // COM to NO (Servo)
  delay(3000);
  Serial.println("Desactivando relé 1");
  digitalWrite(RELAY1, HIGH); // COM to NC

  delay(1000);
  Serial.println("Activando relé 2 (compuerta 2)...");
  digitalWrite(RELAY2, LOW);  // COM to NO (NEMA17)
  delay(3000);
  Serial.println("Desactivando relé 2");
  digitalWrite(RELAY2, HIGH); // COM to NC 
}

void loop() {
  // Nada
}