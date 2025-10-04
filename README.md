# Cyberplátano RD - WRO 2025 Panama 

> Dominican Republic • Future Engineers • 2025

---

## Engineering Documentation
This repository contains all files, codes, and designs used by **Cyberplátano RD** for the **Future Engineers category - WRO 2025 - Panama**.  We document the development of our robot, from mechanical and electronic components to programming and simulation.

---
## Index
- 1. [ Introduction](#introduction)  
- 2. [ Repository Overview](#repository-overview)
- 3. [ Team Members](#team-members)  
- 4. [ Hardware](models/)
      - [Materials List](#materials-list)
      - [Chasis & Mobility](#chassis-&-mobility)
      - [Motors & Drivers](#motors-&-drivers)
      - [Power & Wiring](#power-&-wiring)
      - [Sensors](#sensors)
- 5. [ 3D Design](models/)
      - [Assembly](#assembly)
      - [Final Robot](#final-robot)
- 6. [ Software](src/)
      - [Open challenge](#open-challenge)
      - [Obstacle challenge](#obstacle-challenge)
- 7. [ Schematics](schemes/) 
      - [PCB layout](#pcb-layout)
      - [Diagram](#diagram)
- 8. [ Testing](extras/)
      - ["1st fogueo"](#"1st-fogueo")
      - ["2nd fogueo"](#"2nd-fogueo")
      - ["3rd fogueo"](#"3rd-fogueo")
      - ["4th fogueo"](#"4th-fogueo")
- 9. [ Instructions](extras/)

##  Introduction
This repository documents the journey of Cyberplátano RD, a team representing the Dominican Republic in the World Robot Olympiad (WRO) 2025 – Future Engineers Category, to be held in Panama.

Our goal is to design, build, and program an autonomous robot car capable of completing the competition challenges: the Open Challenge and the Obstacle Challenge. The project integrates mechanical design, electronics, and software, combining skills from 3D printing, circuit design, and programming in C++ and Python.

By sharing our process through this repository, we aim to provide transparency in our work, create a detailed record of our engineering decisions, and contribute to the global robotics community. Every step, from materials and schematics to testing and results, is documented here.


##  Repository Overview
* `extras` - Other essential files
* `images` - Phots and videos 
* `models` - 3D CAD files
* `schemes` - Electrical schematics
* `src` - Main and other programs to run/control software

##  Team Members
* Ivan Saint-Hilaire, ivanrma0702@gmail.com
* Brittany Martinez, martinezp.brtt@gmail.com
* Maria Ramos, ramosbinetm@gmail.com

## Hardware
### Materials List
| Name | Purpose  | Quantity | Price |
| ----------- | ----------- | ----------- |  ----------- |
| Raspberry Pi 5 | Main computer used for high-level processing   |  1  | $98.00    |
| ESP32 | Microcontroller responsible for real-time control of motors and sensors.|  1  | $10.00  |
| Motor Lego NXT | DC motor with encoder used for driving the wheels |  1  | $7.00  |
| NXT Lego Cable | Connects NXT motors and sensors to the control electronics. |  1  | $4.00 |
| 62mm Wheels | Provide traction and mobility to the robot. |  4  | $5.00 |
| H-Bridge (L298N) | Motor driver that allows bidirectional control of the DC motors, enabling forward and backward movement. |  1  | $2.50 |
| Webcam | Captures real-time video feed for visual processing |  1  | $20.00  |
| Servo Motor HS-485HB | Controls steering (front axle movement) or small actuations requiring angular precision. |  1  | $23.00 |
| Ultrasonic Sensor (HC-SR04) | Measures distance to nearby objects for obstacle detection and wall avoidance. |  3  | $4.50 each |
| MPU9250| Inertial Measurement Unit (IMU) that provides accelerometer, gyroscope, and magnetometer data for orientation and navigation. |  1  | $18.00 |
| Switch SPST 6A | Main power switch to safely turn the robot ON/OFF. |  1  | $0.50 |
| Push Button| Used as a start/stop input for robot operation. |  1  | $1.50 |
| Lego Motor Shaft | Mechanical part that transfers torque from the motor to the wheels/axle. |  1  | $0.10 |
| PLA Filament (Polylactic Acid) | Material used for 3D printing the chassis and custom parts of the robot due to its light weight and ease of printing. |  1  | $25.00 |

**Total Car Cost: $219.10US**

---

### Chasis & Mobility
The chassis is 3D-printed using black PLA filament, optimized for both light weight and structural strength. The frame includes pre-designed holes and channels to allow clean and secure wiring, reducing clutter and preventing loose cables from interfering with moving parts. 

- 4 × 65 mm wheels, with rear-wheel drive and front steering controlled by a servo.

- Heavy components (battery, drivers) are placed low and centered to improve stability.

### Motors & Drivers
The robot uses a hybrid actuation system that combines DC motors and servos to achieve both propulsion and precise steering. Propulsion is provided by a **Lego NXT DC** motor with an integrated encoder, which drives the rear wheels and supplies feedback for odometry, allowing the system to estimate both speed and distance traveled. The motor is controlled through an **L298N H-Bridge driver**, which enables bidirectional motion and PWM-based speed regulation. Although not the most power-efficient driver, the L298N was chosen for its reliability, simplicity, and compatibility with 5 V logic levels. Steering is handled by an HS-485HB servo motor mounted on the front axle. This servo provides accurate angular positioning, enabling the robot to perform precise maneuvers on the competition track. Torque is transmitted to the wheels using a custom Lego motor shaft, designed to be modular so that parts can be replaced quickly during testing or competition.

<img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/0ef4ceb7-88f6-4a08-ac8c-b252dd549d79" />

**Motor Lego NXT**	

<img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/c509f6f4-3782-4f07-acd4-867940a67faf" />

**L298N H-Bridge**	


### Power & Wiring
Our robot uses **two separate battery packs**, each consisting of two 18650 Li-ion cells, to power different parts of the system and ensure stable operation.

- Battery Pack 1: Connected to a 5 V 3 A UPS. This pack powers the Raspberry Pi 5, providing uninterrupted power supply to handle high-level processing and computer vision tasks. The UPS ensures that the Pi stays powered even if the main battery momentarily drops or during short interruptions, which is critical during competition.

- Battery Pack 2: Connected to a XL4015 step-down voltage regulator set to 5 V. This pack supplies power to the ESP32, motors, and servo, delivering a stable voltage for real-time control of actuators and sensors. The regulated output ensures that PWM signals and sensor readings remain consistent and reliable.

The chassis is designed with pre-drilled holes and channels for cable management, keeping wires organized and preventing interference with moving parts. Both battery packs are mounted to maintain a low and centered center of gravity, improving stability. A SPST 6 A switch serves as the main power on/off control, and a push button is used to start or stop the robot’s programs.

This dual-battery system separates high-load electronics from control circuits, reducing voltage drops and improving overall reliability during testing and competition runs.

### Sensors

1. **Ultrasonic Sensor (HC-SR04)**  
- **How it works:** Sends out sound waves and measures how long they take to bounce back.  
- **What it does in our robot:**  
  - Detects walls and obstacles.  
  - Helps during the *Obstacle Challenge*.  
- **Connection:** Works with the ESP32 through digital pins (Trig and Echo).  
- **Power:** 5V.  

<img width="500" height="500" alt="HC-SR04 Ultrasonic Sensor" src="https://images.theengineeringprojects.com/image/main/2018/10/Introduction-to-HC-SR04.jpg" />


2. **IMU MPU-9250 (Accelerometer + Gyroscope + Magnetometer)**  

 - **How it works:** Combines accelerometer, gyroscope, and magnetometer data.  
- **What it does in our robot:**  
  - Measures tilt, rotation, and direction.  
  - Keeps the robot driving in a straight line.  
  - Helps smooth out turns.  
- **Connection:** Communicates with the ESP32 using I2C (SDA & SCL).  
- **Power:** 3.3V (⚠️ important: cannot be connected to 5V).  

<img width="500" height="500" alt="MPU-9250 Accelerometer Gyroscope Magnetometer Module Axis Orientation" src="https://protosupplies.com/wp-content/uploads/2019/03/MPU-9250-Accel-Gyro-and-Mag-Module-Axis-Orientation.jpg" />


3. **Webcam**  

- **How it works:** Captures images and sends them to the Raspberry Pi 5.  
- **What it does in our robot:**  
  - Detects lanes, colors, or markers.  
  - Allows real-time testing with visual feedback.  
- **Connection:** USB to Raspberry Pi 5.  
- **Power:** Powered by Raspberry Pi.  

<img width="500" height="500" alt="j5create JVCU100 USB Video Capture Adapter" src="https://info.j5create.com/cdn/shop/products/JVCU100-1-2_2400x.jpg?v=1593027468" />

---

## 3D Design
> CAD and STL files available in [models/](models/).


### Assembly



### Final Robot 
This folder represents the **reference design** for manufacturing and testing.  
 
![Isometric View](./images/Isometric.jpeg)

This is the **second version of the CyberplátanoRD robot** for WRO 2025.  
🚧 The final version is still under construction and will include several upgrades. 🚧


---

## Software

### Open challenge - Preview

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

### Obstacle challenge

---

## Schematics

### Diagram - Preview


### PCB layout - Preview

---

## Testing

### "1st fogueo"
### "2nd fogueo"
### "3rd fogueo"
### "4th fogueo"

---

## Instructions
