int ECHO = 3;
int TRIG = 2;

unsigned long duration;
float distance;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(TRIG,OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  detectarBasura();
}

void detectarBasura(){
  // Enviar un pulso al TRIG_PIN
  digitalWrite(TRIG, LOW);  // Asegurarse de que el TRIG_PIN esté en LOW
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); // Enviar un pulso HIGH
  delayMicroseconds(10);        // Duración del pulso
  digitalWrite(TRIG, LOW);  // Apagar el TRIG_PIN

  // Leer la duración del pulso de vuelta desde el ECHO_PIN
  duration = pulseIn(ECHO, HIGH);  // Medir el tiempo del pulso
  distance = duration * 0.017;

  Serial.print("Distancia: ");
  Serial.print(distance);
  Serial.println(" cm.");
}