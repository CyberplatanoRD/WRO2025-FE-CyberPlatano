# Cyberplátano RD - World Robot Olympiad 2025
---

## Project Documentation
This repository contains all files, codes, and designs used by **Cyberplátano RD** for the **Future Engineers category - WRO 2025**.  
We document the development of our robot, from mechanical and electronic components to programming and simulation.

---

##  Repository Overview
- [ Electronics](/Electronics) → Circuits, schematics, and wiring diagrams  
- [ Mechanics](/Mechanics) → CAD models, mechanical drawings, and assemblies  
- [ Software](/Software) → Source code and control algorithms
- [ Media](/Media) → Photos, videos, and graphical material
- [ Other](/Other) → Other essential files
---

##  Cyberplátano Team 
- **Iván Saint-Hilaire** – ivanrma0702@gmail.com
- **Brittany Martínez** – martinezp.brtt@gmail.com
- **Maria Liz Ramos** – 

---

##  README Contents
- [Project Overview](#-project-overview)  
- [Electronics](#-electronics-)  
- [Mechanics](#-mechanics-)  
- [Software](#-software-)  
- [Connectivity](#-connectivity-)  
- [Object Interaction](#-object-interaction-)  
- [Signal Management](#-signal-management-)  
- [Assembly Guide](#-assembly-guide-)  

---

##  Project Overview 
Our robot is designed to compete in the **Future Engineers category**, following the official WRO 2025 rules.  
It integrates a steering system with a servo, DC motor propulsion, ultrasonic sensors, and an ESP32-CAM for computer vision.

---

##  Electronics 
- Controller: Arduino Nano + ESP32-CAM  
- Power driver: L298N  
- Sensors: HC-SR04 Ultrasonic  
- Power supply: 12V rechargeable battery  
- Servomechanisms for steering  

---

##  Mechanics 
- Chassis designed in CAD  
- Car-like steering system  
- Modular support for sensors and camera  
- Materials: PLA / lightweight aluminum  

---

##  Software 
- Language: C++ (Arduino IDE / PlatformIO)  
- Simulation: Webots  
- Autonomous control algorithms:  
  - Reading ultrasonic sensors  
  - Image processing from ESP32-CAM  
  - Motor and servo control  

---

## Connectivity 
- Serial communication between Arduino Nano and ESP32-CAM  
- Video transmission via ESP32-CAM WiFi  
- Initialization protocol for autonomous startup  

---

## Object Interaction
- Obstacle detection with ultrasonic sensors  
- Trajectory avoidance algorithms  

---

## Signal Management
- Detection of pillars and signals (Obstacle Challenge)  
- Adaptive navigation strategy  

---

##  Assembly Guide 
Step-by-step instructions to assemble the robot (documentation + photos in `/mechanics` and `/electronics`).  

---
