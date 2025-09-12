#include <Wire.h>
#include <ESP32Servo.h>

// ---------- VARIABLES NUEVAS PARA CONTROL DE EXTREMOS ----------
/*
unsigned long tiempoEnExtremo = 0;
bool enExtremo = false;
const unsigned long limiteExtremo = 200; // ms en extremo

unsigned long tiempoEnCentro = 0;
bool enCentro = false;
const unsigned long duracionCentro = 180; // ms en 90°

int ultimoServoAngle = 90; // guarda el último ángulo aplicado
*/

// ---------- PINES ----------
const int trigLeft  = 23;
const int echoLeft  = 25;
const int trigRight = 26;
const int echoRight = 27;



const int servoPin = 18;

const int IN1 = 32; // Motor driver input 1
const int IN2 = 33; // Motor driver input 2

// ---------- PARÁMETROS ----------
const unsigned long usIntervalMs = 100UL; // intervalo de lectura ultras (ms)
const int maxDistance = 80;                // máximo considerado por sensor (cm)
const int diffMax = maxDistance;           // máximo (absoluto) para la diferencia
const int servoCenter = 90;
const int servoLeft   = 45;   // ángulo mínimo
const int servoRight  = 135;  // ángulo máximo

// ---------- VARIABLES ----------
Servo myServo;
unsigned long lastUsRead = 0;

int ultrasonicoizquierda = 0;
int ultrasonicoderecha = 0;
int calculodistancia = 0; // izquierda - derecha

// ---------- PROTOTIPOS ----------
void motorAvanzar();
void motorStop();
void motorReversa();
int leerDistanciaUltrasonico(int trigPin, int echoPin);

// ---------- FUNCIONES MOTOR ----------
void motorAvanzar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void motorReversa() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

// ---------- FUNCION ULTRASONICO ----------
int leerDistanciaUltrasonico(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duracion = pulseIn(echoPin, HIGH, 30000UL);

  if (duracion == 0) {
    return 0; // sin eco → devolvemos 0
  }

  float distanciaF = (duracion * 0.0343f) / 2.0f;
  int distancia = (int)round(distanciaF);
  if (distancia < 0) distancia = 0;

  return distancia;
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(10);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  motorStop();

  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);
  digitalWrite(trigLeft, LOW);
  digitalWrite(trigRight, LOW);

  myServo.attach(servoPin);
  myServo.write(servoCenter);
  delay(200);

  Serial.println("Setup listo. Empezando lecturas (millis non-blocking).");
}

// ---------- LOOP ----------
void loop() {
  unsigned long now = millis();

  if (now - lastUsRead >= usIntervalMs) {
    lastUsRead = now;

    ultrasonicoizquierda = leerDistanciaUltrasonico(trigLeft, echoLeft);
    ultrasonicoderecha  = leerDistanciaUltrasonico(trigRight, echoRight);

    calculodistancia = ultrasonicoizquierda - ultrasonicoderecha;

    if (calculodistancia > diffMax) calculodistancia = diffMax;
    if (calculodistancia < -diffMax) calculodistancia = -diffMax;

    int servoAngle = map(calculodistancia, -diffMax, diffMax, servoLeft, servoRight);

    // ---- CONTROL DE TIEMPO EN EXTREMOS ----
    /*
    if (servoAngle == servoLeft || servoAngle == servoRight) {
      if (!enExtremo) {
        tiempoEnExtremo = now;
        enExtremo = true;
      } else {
        if (now - tiempoEnExtremo >= limiteExtremo) {
          servoAngle = servoCenter; // forzar regreso a 90°
          enExtremo = false;
          tiempoEnCentro = now; // empezar contador de 200 ms en 90°
          enCentro = true;
        }
      }
    } else {
      enExtremo = false;
    }

    // ---- CONTROL DE DURACION EN CENTRO ----
    if (enCentro) {
      if (now - tiempoEnCentro < duracionCentro) {
        servoAngle = servoCenter; // mantener 90°
      } else {
        enCentro = false; // se acabó el tiempo en 90°, seguir normal
      }
    }

    ultimoServoAngle = servoAngle;
    */
    

    myServo.write(servoAngle);

    motorAvanzar();

    Serial.print("Izq: "); Serial.print(ultrasonicoizquierda);
    Serial.print(" cm | Der: "); Serial.print(ultrasonicoderecha);
    Serial.print(" cm | Diff: "); Serial.print(calculodistancia);
    Serial.print(" -> Servo: "); Serial.println(servoAngle);
  }
}
