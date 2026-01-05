# PID Enabled Inverted Pendulum

This repository contains the work done for the ARRT Lab IP Project, focused on the
design and implementation of a PID enabled inverted pendulum using MATLAB
simulation and Arduino-based hardware control.

---

## Objective

To design a PID enabled inverted pendulum.

---

## Tools and Software Used

- MATLAB
- AutoFusion360
- Arduino IDE

---

## Components Used

- 12V Battery
- Arduino Uno
- L298 2A Motor Driver
- MPU6050 IMU
- 5V DC Motor with Encoder (Non-stepper)
- Breadboard
- Jumper Cables
- Soldron 25W

---

## Procedure

### 1. State Space Model

A state-space model is a mathematical representation of a physical system as a set of
input, output, and state variables related by first-order differential equations.
The state variables define the values of the output variables.

The inverted pendulum system was modeled using state-space equations derived
from the dynamics of the pendulum.

---

### 2. Simulation

The derived state-space model was used in MATLAB for simulation.

- ODE45 was used to solve the system equations
- Initial angular position and velocity can be specified
- A PID controller generates the control signal
- The controller moves the pendulum to the vertical position
- An animation of the pendulum motion is generated

Simulation video link is provided in the media folder.

---

## MATLAB Implementation

The MATLAB code:
- Defines system parameters and symbolic variables
- Implements PID control
- Solves the equations using ODE45
- Animates the pendulum motion

---

## PID Controller

A proportional–integral–derivative controller (PID controller) is a control loop
mechanism employing feedback.

- The controller continuously calculates an error value
- A control signal u(t) is generated to reduce the error
- Proportional term reduces the error
- Integral and derivative terms smooth the system response
- Kp, Ki, and Kd determine the controller behaviour

---

## Hardware Implementation

The hardware setup includes:
- Arduino Uno
- MPU6050 IMU
- DC motor with encoder
- Motor driver
- Power supply

The Arduino code:
- Reads IMU and encoder data
- Computes PID control signal
- Controls motor direction and speed
- Uses interrupts for encoder feedback

---

## Hardware Setup

The pendulum bob was 3D printed and the IMU was taped to the bob.

Hardware images are available in the hardware folder.

---

## Repository Structure

matlab/    MATLAB simulation code  
arduino/  Arduino PID control code  
hardware/ Circuit and setup images  
report/   Original ARRT IP report  
media/    Simulation video links  

---

## Notes

This repository is a direct conversion of the ARRT Lab IP report into a structured
GitHub repository for documentation and demonstration.
