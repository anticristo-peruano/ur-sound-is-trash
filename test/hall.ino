int hallPin = A0;   

String detectarPolo(float voltage, float base = 2.5, float margen = 0.1) {
  if (voltage > base + margen) {
    return "norte";
  }
  else if (voltage < base - margen) {
    return "sur";
  }
  else {
    return "nada";
  }
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  int raw = analogRead(hallPin);         
  float voltage = raw * (5.0 / 1023.0);   

  String polo = detectarPolo(voltage);

  Serial.print("Voltaje: ");
  Serial.print(voltage);
  Serial.print(" V\tPolo detectado: ");
  Serial.println(polo);

  delay(200);
}