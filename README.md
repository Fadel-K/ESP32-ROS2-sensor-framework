# ESP32-ROS2 Sensor Framework

A ROS2-based distributed robotics framework that leverages an ESP32 microcontroller as a low-level sensor and actuator interface, coordinated by a high-level ROS node running on a Jetson Orin Nano.

## Overview

This framework provides a bridge between low-level hardware control (sensors and actuators) on an ESP32 and high-level decision making on a Jetson Orin Nano. The ESP32 handles real-time sensor reading and actuator control, while the Jetson Orin Nano runs ROS2 nodes for higher-level logic and coordination.

### Architecture

- **ESP32 (Low-Level)**: Reads sensor data and controls actuators in real-time
- **Jetson Orin Nano (High-Level)**: Runs ROS2 nodes for decision-making and mission planning
- **Communication**: ROS2 messaging between the two devices for sensor data and command exchange

## Project Structure

- `include/` - Header files for the project
- `src/` - Source code files (main firmware for ESP32)
- `lib/` - Library files and dependencies
- `test/` - Test files
- `platformio.ini` - PlatformIO configuration for ESP32 development
- `CMakeLists.txt` - CMake build configuration

## Getting Started

This project uses CMake and PlatformIO for building and deploying ESP32 firmware. The ESP32 communicates with the Jetson Orin Nano via ROS2 to send sensor data and receive actuator commands.

## TODO Tasks

- [ ] **Create a true buffer for TX transmission** - Currently TX transmission blocks and sends data one by one. Need to implement a proper buffering system to handle asynchronous transmission without blocking.

- [ ] **Add external oscillator for accurate timekeeping** - Integrate an external oscillator to improve timing accuracy and stability for time-dependent sensor operations.

- [ ] **Add external oscillator for accurate timekeeping** - Integrate an external oscillator to improve timing accuracy and stability for time-dependent sensor operations.
