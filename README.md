<h1 align="center">CyberPlátanoRD — WRO</h1>

![CyberPlátano Logo](t-photos/cyberplatanord-logo.jpg)

> • Pontificia Universidad Católica Madre y Maestra (PUCMM), Dominican Republic • Future Engineers • 2025

---

<p align="center">
  <a href="https://www.youtube.com/@CyberPlatanoWRO">
    <img src="https://img.shields.io/badge/Youtube-%23FF0000.svg?style=for-the-badge&logo=Youtube&logoColor=white" alt="YouTube" style="border:0;">
  </a>
  <a href="https://www.instagram.com/pucmm?utm_source=ig_web_button_share_sheet&igsh=ZDNlZDc0MzIxNw==">
    <img src="https://img.shields.io/badge/Instagram-%23E4405F.svg?style=for-the-badge&logo=Instagram&logoColor=white" alt="Instagram" style="border:0;">
  </a>
</p>

---

# Engineering Documentation
This repository contains all documentation for the **CyberPlátanoRD** team's robot for the **Future Engineers category - World Robot Olympiad 2025**.  We document the development of our robot, from mechanical and electronic components to programming and simulation.


# Repository Overview

| Folder/File | Description |
|--------------|-------------|
| `models`     | Files for models used by 3D printers to produce the vehicle elements. |
| `other`      | Other essential files. |
| `schemes`    | Schematic diagrams of the electromechanical components used in the vehicle and how they connect to each other. |
| `src`        | Contains code of control software for all components which were programmed to participate in the competition. |
| `t-photos`   | Two photos of the team. |
| `v-photos`   | Contains six photos of the vehicle (from every side, from top and bottom). |

# Table of Contents
* [Introduction](#introduction)  
* [About Us](#part-1-about-us)
* [Mobility Management](#part-2-mobility-management)  
* [Power and Sense Management](#part-3-power-and-sense-management)  
* [Obstacle/Open Management](#part-4-obstacleopen-management)

---

# Introduction
This repository documents the journey of Cyberplátano RD, a team representing the Dominican Republic in the World Robot Olympiad (WRO) 2025 – Future Engineers Category, to be held in **Singapore**.

**Our goal** is to design, build, and program an autonomous robot car capable of completing the competition challenges: the Open Challenge and the Obstacle Challenge. The project integrates mechanical design, electronics, and software, combining skills from 3D printing, circuit design, and programming in C++ and Python.

By sharing our process through this repository, **we aim** to provide transparency in our work, create a detailed record of our engineering decisions, and contribute to the global robotics community. Every step, from materials and schematics to testing and results, is documented here.

## Documentation Video

---

# Part 1: About Us

## Our Team 
We are CyberPlátano, representing the Caribbean, Dominican Republic. Part of the PUCMM Robotics Club and guided by our mentor Álvaro Zapata, we are a group of young future engineers. Our club first competed in WRO 2023 in Panamá, and this year we continue our journey, bringing innovation, teamwork, and passion to the international stage in Singapore.

Our team consists of three Mechatronics Engineering students from  "Pontificia Universidad Católica Madre y Maestra (PUCMM)" currently in their third semester. We met during our studies and decided to collaborate as teammates, driven by our shared interest in robotics and innovation. Throughout our academic journey, we have gained valuable experience from our university projects and from the guidance of our professors and peers. This collaboration and learning process have led us to participate together on this international stage.

---

##  Team Members

#### 1. Ivan Saint-Hilaire - Software / Mechanical Eng. ####
| Photo | Role |
|-------|------|
| <img src="image.png" width="200"> | Iván is responsible for designing and optimizing the mechanical components of the robot while also developing and integrating the software that controls its functions. His expertise ensures that the robot operates efficiently, accurately, and reliably, bridging the gap between hardware and software.|


#### 2. Brittany Martinez - Team Coordinator and Document Designer  ####
| Photo | Role |
|-------|------|
| <img src="image.png" width="200"> | Brittany is responsible for documentation. She has skills in electronics and programming, which allow her to keep the team’s processes well organized. She also ensures that all procedures and systems the team works on are carried out accurately and correctly, maintaining a clear workflow.|

#### 3. Maria Liz Ramos  - Electronics Eng. ####
| Photo | Role |
|-------|------|
| <img src="image.png" width="200"> | María is responsible for designing, implementing, and maintaining the robot’s electronic systems, ensuring all sensors, circuits, and components function accurately and reliably for optimal performance in the competition.|

#### Alvaro Zapata  - Coach ####
| Photo | Role |
|-------|------|
| <img src="image.png" width="200"> | Alvaro is a Mechatronics Engineer, he mentors the team, providing technical guidance, strategic advice, and support to ensure the robot performs efficiently and the team works effectively.|

---

##  Work Schedule







---

# Part 2: Mobility Management
---
## Robot's Chassis 
Our robot’s chassis has been completely designed and developed from scratch, using Fusion 360 and 3D-printed in black PLA. The design philosophy is centered around modularity, balance, and accessibility, allowing easy modifications, component replacements, and upgrades throughout the development cycle.

We structured our chassis with three essential parts, each designed to provide both strength and modularity to the overall frame:

**Main Base:** the central foundation that holds the motor driver, batteries, and wiring paths.

**Back Support**: provides mechanical rigidity and ensures structural stability when handling torque or collisions.

**Standoff & Upper Base**: elevates the ESP32-CAM and sensors, optimizing field of view and cooling while keeping power and logic electronics separated.

Each element interlocks seamlessly, resulting in a lightweight yet durable frame. Cable management channels were integrated into the design to maintain a clean wiring layout and prevent interference with moving parts. The modular assembly also allows future versions to incorporate sensor mounts or additional mechanical features without redesigning the entire frame.

During the design process, special attention was given to:

- **Weight distribution**, ensuring the center of gravity remains low and centered.

- **Thermal management**, by maintaining airflow under the electronics.

- **Maintenance accessibility**, allowing any component to be removed within minutes.

All the 3D printable models can be found [here](#models).

---

# Part 3: Power and Sense Management


# Part 4: Obstacle/Open Management
> CPP and Python codes files available in [src/](src/).

### Open challenge - Preview

Below is a short preview of the control logic used in our Open Challenge robot.  

The full version is available here  [`src/open_challengev2.ino`](./src/open_challengev2.ino)

```cpp
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <ESP32Servo.h>

MPU9250_asukiaaa mySensor;
Servo servoMotor;

// Basic yaw control variables
float yaw = 0.0;
float gyroZ_offset = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mySensor.setWire(&Wire);
  mySensor.beginGyro();
  servoMotor.attach(18);
}

void loop() {
  mySensor.gyroUpdate();
  yaw += (mySensor.gyroZ() - gyroZ_offset) * 0.01;
  servoMotor.write(map((int)yaw % 360, 0, 360, 50, 130));
}

// ...rest of preview code...

```
---

### Obstacle challenge

---

## Schematics
> Diagrams and PCB Layout files available in [schemes/](schemes/).

### Diagram - Preview
[View full PDF](./schemes/WRO_Generalsquematic.pdf)

![Schematic Preview](./schemes/Generalsquematicpreview.png)

### PCB layout - Preview
[View full PCB Layout](./schemes/WRO_Generalpcblayout.pdf)


---

## Testing
The **"fogueos"** are an internal dynamic organized by the teams from PUCMM (Pontificia Universidad Católica Madre y Maestra), where participants from WRO Dominican Republic take part in friendly mini competitions.

The main goal is to test the robots under competition-like conditions, evaluate team performance, and share knowledge and improvements among teams.

Throughout this process, our team participated in several fogueos to enhance the robot’s precision, stability, and overall performance:

### "1st fogueo"
### "2nd fogueo"
### "3rd fogueo"
### "4th fogueo"
### "5th fogueo"
### "6th fogueo"
### "7th fogueo"
---

## Instructions
