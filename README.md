# Custom STM32 Flight Controller

A custom flight controller project based on an **STM32F405** microcontroller.

The long-term goal of this project is to develop a quadcopter flight controller from the ground up, starting with the STM32 hardware and gradually implementing sensor communication, flight stabilization, radio control, motor control, and other flight-control systems.

> **Project Status: Early Development**
>
> The STM32 development environment has been successfully configured and the microcontroller can be programmed and debugged using ST-LINK.

---

## Project Goals

This project is intended to explore embedded systems and flight-control development by building the system from the hardware level upward.

Planned areas of development include:

- STM32 embedded programming
- Peripheral configuration
- SPI/I²C communication
- IMU integration
- Sensor calibration
- Attitude estimation
- PID control
- Radio communication
- Motor control
- ESC communication
- Flight-control algorithms
- Custom drone hardware
- 3D-printed airframe design

---

## Current Hardware

### Microcontroller

- **STM32F405**
- ARM Cortex-M4
- 1 MB Flash
- STM32 HAL firmware framework

### Programmer / Debugger

- **ST-LINK**
- SWD debugging/programming

### Development Software

- STM32CubeMX
- STM32CubeIDE
- STM32CubeProgrammer

## Other Drone Hardware
- 4IN1 60A ESC 6S
- 1100mAh LiPo Battery 6S
- 4x iFlight XING E Pro 2207 Brushless Motor
- ICM-20602 IMU
  
---

## Current Status

### STM32 Setup

- [x] STM32 project created
- [x] MCU configured in STM32CubeMX
- [x] GPIO initialized
- [x] Project generated for STM32CubeIDE
- [x] STM32CubeIDE project builds
- [x] ST-LINK detected
- [x] STM32 successfully programmed through ST-LINK
- [x] Debugging connection established
- [ ] External sensors connected
- [ ] Flight-control software implemented

---

## Development Environment

The project currently uses STM32CubeMX to configure the microcontroller and generate the initial HAL-based project.

STM32CubeIDE is used for:

- Writing firmware
- Building the project
- Flashing the STM32
- Debugging
- Inspecting variables
- Stepping through code

ST-LINK is used as the SWD programmer and debugger.
