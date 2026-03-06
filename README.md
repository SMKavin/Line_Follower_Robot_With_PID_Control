# PID Line Follower Robot

An autonomous **PID-based line follower robot** built using an Arduino-compatible microcontroller, IR sensor array, and an **L298N motor driver**. The robot detects a line on the ground and continuously adjusts motor speeds using a **PID control algorithm** to maintain smooth and stable tracking.

---

## Overview

This project demonstrates the implementation of closed-loop control in robotics. A set of infrared sensors detect the position of a line relative to the robot. The microcontroller processes these readings to calculate an error value that represents how far the robot is from the center of the line.

A **PID (Proportional–Integral–Derivative) controller** is used to minimize this error. Based on the computed correction, the speeds of the left and right motors are dynamically adjusted using PWM signals through an L298N motor driver.

This allows the robot to perform smooth turns and maintain stable motion while following the path.

---

## Features

- PID-based control for smooth line tracking
- Automatic sensor calibration
- Adjustable PID constants
- Speed ramp-up for stable acceleration
- Works with **black or white lines**

---

## Hardware Components

- Arduino / compatible microcontroller
- L298N Motor Driver
- IR Sensor Array (5 or 7 sensors)
- 2 × DC Motors
- Robot chassis
- Battery pack
- Push buttons for calibration/start
- Indicator LED

---

## Control Algorithm

The robot calculates the **position error** based on sensor readings and applies PID correction.

PID equation used:

PID = Kp * P + Ki * I + Kd * D

Where:

- **P (Proportional)** – responds to the current error
- **I (Integral)** – accumulates past error
- **D (Derivative)** – predicts future error based on rate of change

Motor speeds are then adjusted:

LeftSpeed = BaseSpeed - PID
RightSpeed = BaseSpeed + PID

## PID Parameters

Kp = 0.08
Kd = 0.15
Ki = 0

## Calibration Process

1. Press the first button to start calibration
2. The robot rotates to scan the surface
3. Minimum and maximum sensor values are recorded
4. Threshold values are automatically computed
5. Press the second button to start line following

## How It Works

1. Sensors read line reflectance values
2. Sensor data is normalized using calibration values
3. Error is calculated using sensor weights
4. PID controller computes correction
5. Motor speeds are adjusted using PWM signals
6. Robot continuously corrects its path
