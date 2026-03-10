# BLDC Motor Control Project

This is an ongoing project to control a Brushless DC (BLDC) motor. My current focus is on driving the motor and managing its speed through a microcontroller.

## Current Project Goal
The main objective right now is to:
- Drive a BLDC motor using a dedicated motor driver.
- Set specific **RPM** values to control the rotation speed.
- Use the **CAN Bus** module for communication and sending control commands.

## Materials
- **Microcontroller:** STM32F429
- **Motor:** Brushless DC (BLDC) Motor
- **Driver:** 3-Phase Motor Driver
- **Communication:** CAN Module
- **Power:** DC Power Supply

## Project Status
This project is currently under development. The core driving logic and RPM control via CAN are the main features I am working on right now. More updates and features will be added as the project progresses.

## How to Use
1. Set up the hardware connections.
2. Compile and flash the code using STM32CubeIDE.
3. Send speed commands to reach the desired RPM.
