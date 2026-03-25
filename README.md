# BLDC Motor Control with STM32

## 📋 Project Overview
This project focuses on the precise control and synchronization of BLDC motors using the STM32F429 microcontroller. The primary goal is to achieve balanced movement and high-efficiency performance through synchronized motor operation.

## 🛠 Hardware Components
Microcontroller: STM32F429

Actuators: 2x Synchronized BLDC Motors

Communication: USB/ACM interface for real-time monitoring and control.

## 🚀 Key Features
Real-time Synchronization: Coordinated control of multiple motor units.

Control Algorithms: Implementation of speed and position control.

Data Logging: Instant data tracking via serial communication.

## ⚠️ Project Update: Hardware Revision
Initially, the project was designed to utilize three BLDC motors. However, during the testing phase and a detailed review of the motor datasheets, it was discovered that one of the motors possessed a different gear ratio compared to the others.

Since inconsistent gear ratios lead to discrepancies in speed and torque under the same control signals—making precise synchronization impossible—the system has been optimized to proceed with two fully compatible BLDC motors to ensure operational stability and accuracy.
