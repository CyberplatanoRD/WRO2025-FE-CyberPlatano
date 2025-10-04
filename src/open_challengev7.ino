#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <ESP32Servo.h> // Agregado para controlar servo

MPU9250_asukiaaa mySensor;
Servo servoMotor;       

// Solo variables necesarias para Yaw, gyroZ_raw y gyroZ_f
float yaw = 0.0;
float gyroZ_raw = 0.0;
float gyroZ_f = 0.0;      // gyroZ filtrado
float gyroZ_offset = 0.0; // offset calculado en calibración

// Tiempo
unsigned long prevTime = 0;
float dt = 0.0; // segundos

// Parámetros
const int CAL_SAMPLES = 200;     // lecturas para calibrar gyro Z
const float GYRO_ALPHA = 0.92;   // EMA alpha

//Variables paro de motor
int giroCount = 0;                 // Contador de giros realizados
const int maxGiros = 12;           // Número de giros hasta detener el vehículo
bool vehiculoDetenido = false;     // Bandera para saber si ya se detuvo

unsigned long stopMotorTime = 0;       // momento en que se debe detener el motor
const unsigned long stopDelay = 1000;  // 1 segundo en ms
bool pararMotorPendiente = false;      // indica que se debe parar después de delay

// First corner detection
bool firstCornerDetected = false;
bool firstCornerRight = false; // true if first corner was right, false if left



// ===== Pin del servo =====
const int servoPin = 18;

const int in1 = 32;
const int in2 = 33;

// === Pines ultrasónicos ===
const int trigLeft  = 14;
const int echoLeft  = 27;
const int trigRight = 26;
const int echoRight = 25;

const unsigned long usIntervalMs = 100UL; 
int ultrasonicoizquierda = 0;
int ultrasonicoderecha = 0;

unsigned long lastUsRead = 0; 

// === Banderas para controlar ajustes de yaw ===
bool ajustadoDerecha = false;
bool ajustadoIzquierda = false;

// ===== NUEVAS VARIABLES GLOBALES =====
unsigned long lastYawAdjustTime = 0;        // momento del último ajuste
const unsigned long yawBlockTime = 1000;    // tiempo de bloqueo en ms (1 segundo)


// --- Función para leer distancia ---
int leerDistanciaUltrasonico(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duracion = pulseIn(echoPin, HIGH, 30000UL);

  if (duracion == 0) {
    return -1; // -1 = sin eco
  }

  float distanciaF = (duracion * 0.0343f) / 2.0f;
  int distancia = (int)round(distanciaF);
  if (distancia < 0) distancia = 0;

  return distancia;
}

void motorAvanzar() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void motorDetener() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  vehiculoDetenido = true;
}


void setup() {
  Serial.begin(115200);
  Wire.begin();

  mySensor.setWire(&Wire);
  mySensor.beginGyro();

  // ===== Inicializar servo =====
  servoMotor.attach(servoPin);
  servoMotor.write(90); // posición inicial

  // ===== Inicializar motor =====
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Pines ultrasónicos
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  delay(100);

  // Calibración simple del gyro Z
  Serial.println("Calibrando gyro Z: mantener sensor quieto...");
  double gz_sum = 0.0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    mySensor.gyroUpdate();
    gz_sum += mySensor.gyroZ();
    delay(5);
  }
  gyroZ_offset = gz_sum / CAL_SAMPLES;
  Serial.print("Offset gyroZ: "); Serial.println(gyroZ_offset, 6);

  // Espera después de calibración para estabilizar
  delay(2000);

  // Arranca motor después de calibración
  motorAvanzar();

  prevTime = micros();
}

void loop() {
  // dt real
  unsigned long currTime = micros();
  dt = (currTime - prevTime) / 1000000.0;
  if (dt <= 0.0) dt = 0.000001;
  prevTime = currTime;

  // actualizar giroscopio
  mySensor.gyroUpdate();

  // leer gyroZ y quitar offset
  gyroZ_raw = mySensor.gyroZ() - gyroZ_offset; 

  // EMA para suavizar (gyroZ_f)
  static bool first = true;
  if (first) {
    gyroZ_f = gyroZ_raw;
    first = false;
  } else {
    gyroZ_f = GYRO_ALPHA * gyroZ_f + (1.0 - GYRO_ALPHA) * gyroZ_raw;
  }

  // integrar yaw
  yaw += gyroZ_f * dt; 

  // ---- Ajuste de referencia inicial ----
  static bool firstYaw = true;
  if (firstYaw) {
    yaw = 90.0;   
    firstYaw = false;
  }

  // Normalizar yaw [0, 360)
  if (yaw >= 360) yaw -= 360;
  if (yaw < 0) yaw += 360;

  // === Redondear yaw a enteros ===
  int yawInt = round(yaw);

  // ===== Mapeo de yaw a servo =====
  int servoAngle = map(yawInt, 45, 135, 50, 130);
  servoAngle = constrain(servoAngle, 50, 130);
  servoMotor.write(servoAngle);

  // ===== Leer ultrasónicos cada cierto intervalo =====
 // ===== Leer ultrasónicos cada cierto intervalo =====
if (millis() - lastUsRead >= usIntervalMs) {
    lastUsRead = millis();

    // Read sensors
    if (!firstCornerDetected || !firstCornerRight) { // Read left only if first corner is not right
        ultrasonicoizquierda = leerDistanciaUltrasonico(trigLeft, echoLeft);
    }
    if (!firstCornerDetected || firstCornerRight) { // Read right only if first corner is not left
        ultrasonicoderecha   = leerDistanciaUltrasonico(trigRight, echoRight);
    }

    unsigned long now = millis();

    // ===== Lógica de ajuste con bloqueo temporal y contador de giros =====
    if (!vehiculoDetenido && (now - lastYawAdjustTime >= yawBlockTime)) {

        // --- Izquierda ---
        if (!firstCornerDetected || !firstCornerRight) {  // Only if allowed
            if (ultrasonicoizquierda > 150) {
                if (!ajustadoIzquierda) {
                    yaw += 89; 
                    ajustadoIzquierda = true;
                    lastYawAdjustTime = now;
                    giroCount++;
                    Serial.print("\t>>> Ajuste -90° (Izquierda)");

                    // First corner detection
                    if (!firstCornerDetected) {
                        firstCornerDetected = true;
                        firstCornerRight = false; // first was left
                    }

                    // Motor stop logic
                    if (giroCount >= maxGiros && !pararMotorPendiente) {
                        stopMotorTime = now;
                        pararMotorPendiente = true;
                    }
                }
            } else if (ultrasonicoizquierda == -1) {
                Serial.print("\t(Sin eco izquierda)");
            } else {
                ajustadoIzquierda = false; 
            }
        }

        // --- Derecha ---
        if (!firstCornerDetected || firstCornerRight) { // Only if allowed
            if (ultrasonicoderecha > 150) {
                if (!ajustadoDerecha) {
                    yaw -= 89; 
                    ajustadoDerecha = true;
                    lastYawAdjustTime = now;
                    giroCount++;
                    Serial.print("\t>>> Ajuste +90° (Derecha)");

                    // First corner detection
                    if (!firstCornerDetected) {
                        firstCornerDetected = true;
                        firstCornerRight = true; // first was right
                    }

                    // Motor stop logic
                    if (giroCount >= maxGiros && !pararMotorPendiente) {
                        stopMotorTime = now;
                        pararMotorPendiente = true;
                    }
                }
            } else if (ultrasonicoderecha == -1) {
                Serial.print("\t(Sin eco derecha)");
            } else {
                ajustadoDerecha = false; 
            }
        }
    }

    // Motor stop after delay
    if (pararMotorPendiente && (millis() - stopMotorTime >= stopDelay)) {
        motorDetener();
        pararMotorPendiente = false;
        Serial.println(">>> Motor detenido después de 1 segundo");
    }

    Serial.println();
}


  // Imprimir info de IMU
  Serial.print("Yaw: "); Serial.print(yawInt);
}
