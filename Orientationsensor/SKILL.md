---
name: stm32-imu-requirements-interviewer
description: Use this skill when the user wants to define, clarify, or refine requirements for an STM32 IMU/AHRS firmware project before implementation. Especially relevant for MPU6500, LIS3MDL, DPS310, GNSS, USB FS, calibration, quaternion/Euler output, and firmware architecture planning.
---

# STM32 IMU Requirements Interviewer

## Purpose

This skill helps Codex interview the user before writing firmware.

The goal is to collect complete requirements for an STM32-based IMU/AHRS project, detect unclear assumptions, and produce a usable firmware requirement specification.

Do not write implementation code until the requirements are clear enough.

## Project Context

The project is an STM32-based **Motion Orientation Unit**.

Current hardware context:

- STM32 microcontroller
- MPU6500 accelerometer + gyroscope
- LIS3MDL magnetometer
- DPS310 barometric pressure sensor
- L76L-M33 GNSS module
- USB FS connection to PC

Current firmware direction:

- Read sensors reliably
- Calibrate sensors properly
- Estimate orientation
- Output quaternion internally or externally as required
- Output Euler angles for display/debug when needed
- Provide relative altitude from the barometer
- Communicate with a PC through USB FS
- Support PC commands for calibration, status, and configuration

Current project scope:

- Focus on orientation and relative altitude
- Do not assume full GNSS-based navigation
- Do not assume accurate local XYZ position tracking
- Do not assume lever-arm compensation is required unless the user confirms accurate linear acceleration at a reference point is needed

## Core Behavior

Before writing code, interview the user step by step.

Ask questions in small groups.

Start with the most important unknowns first.

After each answer, update the working understanding and ask the next focused group of questions.

Do not dump every possible question at once.

Do not silently invent missing details.

If the user says "you decide", choose a practical default and clearly mark it as an assumption.

If the user gives uncertain answers, offer two or three practical options and ask them to choose.

## Interview Flow

### 1. Project Goal

Clarify:

- What is the main purpose of the device?
- Is this a prototype, production firmware, research tool, or test firmware?
- What output is required first?
- Who or what will consume the output?
- Is the device mainly stationary, gimbal-like, handheld, mounted on a moving platform, or used for navigation?

Key decision:

- Raw sensor streaming only
- Orientation estimation
- Relative altitude
- PC visualization/debug
- Future navigation expansion

### 2. Hardware Platform

Clarify:

- Exact STM32 part number
- Clock configuration
- STM32CubeIDE, Keil, or other toolchain
- HAL, LL, bare-metal, FreeRTOS, or mixed style
- Existing CubeMX project or new project
- Existing PCB or development board
- Pin assignments
- Bus assignments
- DMA usage
- interrupt lines
- available non-volatile memory for calibration storage

Do not invent pin assignments.

### 3. Sensor Interface Requirements

For each sensor, clarify interface and role.

#### MPU6500

Ask:

- SPI or I2C?
- Chip select pin if SPI?
- Interrupt pin used or not?
- Gyroscope full-scale range?
- Accelerometer full-scale range?
- Output data rate?
- Digital low-pass filter setting?
- FIFO or direct register read?
- Polling, interrupt, DMA, or RTOS task?
- Required timestamp accuracy?
- Mounting orientation relative to device axes?
- Is the sensor away from the mechanical center of rotation?

#### LIS3MDL

Ask:

- SPI or I2C?
- Output data rate?
- Full-scale range?
- Continuous mode or triggered mode?
- Is absolute heading required?
- Is the sensor near motors, magnets, steel, or high-current cables?

#### DPS310

Ask:

- SPI or I2C?
- Pressure and temperature oversampling?
- Output rate?
- Relative altitude requirement?
- Reference pressure or reference altitude source?

#### L76L-M33 GNSS

Ask:

- Is GNSS used now or reserved for later?
- UART baud rate?
- NMEA only or binary protocol?
- Required GNSS output fields?
- Should GNSS affect orientation output now?

### 4. Calibration Requirements

Calibration must not be skipped.

Clarify for each sensor:

- Which calibration is required?
- When should calibration run?
- Startup, factory, user-commanded, or periodic?
- Should calibration values be stored in flash?
- What happens if calibration data is missing or invalid?
- How should calibration status be reported to PC?

Required IMU/AHRS calibration topics:

- Gyroscope zero-rate offset calibration
- Accelerometer offset calibration
- Accelerometer scale calibration
- Accelerometer six-position calibration if needed
- Magnetometer hard-iron calibration
- Magnetometer soft-iron calibration
- Magnetic disturbance detection
- Barometer reference pressure or altitude calibration
- Axis alignment correction
- Temperature drift consideration
- Calibration data validation
- Calibration data storage format

Calibration storage should consider:

- magic value
- version
- sensor identifier
- calibration payload
- validity flag
- CRC/checksum

### 5. Orientation and Fusion Requirements

Clarify:

- Raw data only or AHRS output?
- Quaternion output required?
- Euler output required?
- Angular velocity output required?
- Linear acceleration output required?
- Absolute heading required?
- Magnetometer included in fusion or only logged?
- Relative altitude included in output?
- Target update rate?
- Acceptable drift?
- Stationary stability expectation?

Guidance:

- Prefer quaternion internally for orientation.
- Provide Euler angles for human-readable display/debug.
- Do not design GNSS/local-coordinate navigation unless explicitly requested.

### 6. USB Output and Command Protocol

Clarify:

- USB class: CDC, HID, vendor class, or composite?
- One-way streaming or bidirectional command protocol?
- CSV, JSON, binary packet, or custom framed packet?
- Required update rate?
- Timestamp source and unit?
- Packet checksum or CRC?
- Debug output separation?
- PC application requirement?

Recommended command categories:

- device info
- sensor status
- start streaming
- stop streaming
- set output rate
- start calibration
- read calibration
- save calibration
- erase calibration
- reset device

Recommended packet fields for binary mode:

- sync/header
- protocol version
- packet type
- payload length
- sequence number
- timestamp
- status flags
- payload
- CRC/checksum

### 7. Firmware Architecture

Clarify:

- Existing project structure or new structure?
- HAL/LL/bare-metal preference?
- FreeRTOS or super-loop?
- polling, interrupt, DMA, or task-based sensor reading?
- logging requirement?
- command parser requirement?
- configuration manager requirement?
- error code convention?
- unit test or mock driver requirement?

Recommended layers:

- board support layer
- sensor driver layer
- sensor service layer
- calibration layer
- fusion algorithm layer
- USB protocol layer
- application layer

Recommended states:

- BOOT
- SENSOR_INIT
- CALIBRATION_LOAD
- IDLE
- STREAMING
- CALIBRATING
- FAULT

### 8. Reliability and Fault Handling

Clarify behavior for:

- sensor missing
- wrong WHO_AM_I
- bus timeout
- bad sensor data
- invalid calibration CRC
- USB disconnect
- packet overflow
- watchdog reset
- magnetometer disturbance
- GNSS unavailable

Ask whether the firmware should continue in degraded mode if one sensor fails.

### 9. Acceptance Criteria

Define how the user will know the milestone is complete.

Possible acceptance tests:

- USB device enumerates on PC
- MPU6500 WHO_AM_I returns expected value
- raw accelerometer and gyro data stream correctly
- gyro output is near zero when stationary after calibration
- accelerometer magnitude is close to 1 g when stationary
- LIS3MDL data changes smoothly when rotated
- DPS310 pressure and temperature are reasonable
- relative altitude changes reasonably with height change
- calibration command completes and stores values
- invalid calibration data is detected by CRC/checksum
- quaternion output is stable when stationary
- Euler output does not jump unexpectedly during normal movement
- PC receives packets at target rate
- packet checksum detects corruption

## Final Output Format

After enough information has been collected, produce a document with this structure:

1. Project Summary
2. Confirmed Requirements
3. Open Questions
4. Assumptions
5. Hardware Interface Table
6. Sensor Driver Requirements
7. Calibration Requirements
8. Orientation/Fusion Requirements
9. USB Protocol Requirements
10. Firmware Architecture
11. Error Handling Strategy
12. Calibration Data Storage Format
13. Test Plan
14. Implementation Phases

## Important Rules

Do not write final implementation code during the interview phase.

Do not skip calibration questions.

Do not assume GNSS is active.

Do not assume local XYZ position tracking is required.

Do not assume lever-arm compensation is required.

Do not mix binary streaming and debug text without asking.

Do not place all firmware logic in `main.c`.

Do not invent hardware pin assignments.

Always separate confirmed requirements from assumptions.
