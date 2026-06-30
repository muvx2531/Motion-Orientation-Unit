# Motion Orientation Unit

## 1. Project Overview

Motion Orientation Unit is an STM32-based IMU/AHRS device.

The device reads motion and environmental sensors, performs calibration, estimates orientation, and sends real-time data to a computer through USB FS.

The current focus is orientation and relative altitude, not full GNSS-based navigation or XYZ position tracking.

## 2. Project Goals

- Read MPU6500 accelerometer and gyroscope data
- Read LIS3MDL magnetometer data
- Read DPS310 pressure and temperature data
- Support L76L-M33 GNSS module for future use
- Estimate orientation using quaternion internally
- Provide Euler angles for debugging or display
- Send data to PC through USB FS
- Support calibration commands from PC
- Store calibration data in non-volatile memory

## 3. Hardware

| Component | Function | Interface |
|---|---|---|
| STM32 MCU | Main controller | - |
| MPU6500 | Accelerometer + Gyroscope | SPI or I2C |
| LIS3MDL | Magnetometer | SPI or I2C |
| DPS310 | Barometric pressure sensor | SPI or I2C |
| L76L-M33 | GNSS module | UART |
| USB FS | PC communication | USB CDC or custom protocol |

## 4. Current System Scope

In the current phase, the system should focus on:

- Sensor drivers
- Sensor calibration
- Orientation output
- Relative altitude
- USB communication
- Firmware architecture
- Test and validation

The system should not yet focus on:

- Full global navigation
- Local XYZ position tracking
- GNSS-based coordinate system
- Advanced lever-arm compensation unless required later

## 5. Important Engineering Notes

- Quaternion should be used internally for orientation.
- Euler angles may be output for human-readable display.
- Calibration is required and must not be skipped.
- MPU6500 may not be mounted at the exact mechanical center of rotation.
- For orientation output, this is usually acceptable.
- For accurate linear acceleration at another reference point, lever-arm compensation may be considered later.

## 6. Firmware Architecture

Recommended firmware layers:

- Board support layer
- Sensor driver layer
- Sensor service layer
- Calibration layer
- Sensor fusion layer
- USB protocol layer
- Application layer

## 7. Calibration Scope

The firmware should support:

- Gyroscope zero-rate offset calibration
- Accelerometer offset calibration
- Accelerometer scale calibration
- Magnetometer hard-iron calibration
- Magnetometer soft-iron calibration
- Barometer reference calibration
- Calibration data storage
- Calibration validity check

## 8. USB Communication

USB FS will be used for communication with a PC.

Early development may use USB CDC.

The protocol should eventually support:

- Real-time data streaming
- Device status query
- Calibration command
- Configuration command
- Error reporting

## 9. Development Status

Current phase:

- Requirement definition
- Sensor driver planning
- Calibration workflow planning
- USB output protocol planning

## 10. Acceptance Criteria

The first firmware milestone is complete when:

- USB device enumerates correctly on PC
- MPU6500 WHO_AM_I can be read
- Raw accelerometer and gyroscope data are valid
- Calibration command works
- Calibration values can be stored and loaded
- Orientation output is stable when the device is stationary
- PC receives data at the required update rate