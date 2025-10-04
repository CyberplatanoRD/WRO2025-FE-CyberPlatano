# Source Code

This folder contains the source code for the Open Challenge and Obstacle Challenge robot of the CyberPlátanoRD team for WRO 2025.

## Files included

- `open_challengev2.ino` – Main Arduino sketch for the robot.
  - Handles motor control, yaw adjustment, servo steering, and ultrasonic sensors.
  - Implements corner detection logic and motor stop after a number of rotations.

## Notes

- The code is designed for ESP32 with MPU9250 sensor, servo motor, L298N driver, and ultrasonic sensors.
- Calibration of the gyroscope is performed in the `setup()` function.
- The main loop integrates sensor readings and controls motors and servo based on yaw and ultrasonic distances.

## How to use

1. Open the `.ino` file in Arduino IDE or VS Code with PlatformIO.
2. Make sure all required libraries are installed:
   - `MPU9250_asukiaaa`
   - `ESP32Servo`
3. Connect the ESP32 and upload the sketch.
4. Adjust pins if needed for your hardware setup.

