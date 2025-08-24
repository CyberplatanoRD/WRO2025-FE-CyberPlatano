#include <ESP32Servo.h>

// === Pines Servo ===
Servo myServo;
const int servoPin = 4;

// === Pines Motor DRV8871 ===
const int IN1 = 32;
const int IN2 = 33;

// === Pin del push button ===
const int buttonPin = 25; // Cambia al GPIO que uses
int lastButtonState = HIGH;
int buttonState;

// === Funciones motor ===
void motorAvanzar(int velocidad) {
  digitalWrite(IN1, HIGH);       
  analogWrite(IN2, velocidad);   
}

void motorStop() {
  analogWrite(IN2, 0);
  digitalWrite(IN1, LOW);
}

// === Funciones servo ===
void girar() {
  for (int pos = 90; pos >= 45; pos--) {
    myServo.write(pos);
    motorAvanzar(255);
    delay(1);
  }
}

void enderezar() {
  for (int pos = 45; pos <= 90; pos++) {
    myServo.write(pos);
    motorAvanzar(255);
    delay(1);
  }
}

// === Ejecutar secuencias ===
  // === Secuencia 1 ===
 

void setup() {
  Serial.begin(115200);

  myServo.attach(servoPin);
  myServo.write(90);
  delay(500);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP); // Botón entre GPIO y GND

  Serial.println("Listo. Presiona el botón para iniciar.");
}

void loop() {

  delay(5000);
 motorAvanzar(240);
  delay(1700);
  girar();
  delay(800);
  enderezar();
  delay(800);

  // === Secuencia 2 ===
  motorAvanzar(240);
  delay(1700);
  girar();
  delay(800);
  enderezar();
  delay(800);

  // === Secuencia 3 ===
  motorAvanzar(240);
  delay(1700);
  girar();
  delay(800);
  enderezar();
  delay(800);

  // === Secuencia 4 ===
  motorAvanzar(240);
  delay(1700);
  girar();
  delay(800);
  enderezar();
  delay(800);

  // === Secuencia 5 ===
  motorAvanzar(240);
  delay(1700);
  girar();
  delay(800);
  enderezar();
  delay(800);

  // === Secuencia 6 ===
  motorAvanzar(240);
  delay(1700);
  girar();
  delay(800);
  enderezar();
  delay(800);

  motorStop();
  Serial.println("Secuencia completa.");
}