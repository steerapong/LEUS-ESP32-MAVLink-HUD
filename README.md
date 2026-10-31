# LEUS ESP32-S3 MAVLink HUD
**An Open-Source Experimental UAV Flight Display**
![LEUS ESP32-S3 MAVLink HUD](images/mavlink.jpg)
Developed by **LEADING EDGE UNMANNED SYSTEMS (LEUS)**

A lightweight UAV flight display built with an ESP32-S3 and a 5-inch RGB LCD. The system receives MAVLink telemetry from an ArduPilot flight controller via Wi-Fi UDP and displays real-time flight information.

## Features

- Artificial Horizon with Roll and Pitch indicators
- Roll, Pitch, and Yaw telemetry
- Ground Speed and Relative Altitude
- Battery Voltage and GPS Status
- Flight Mode and Armed/Disarmed Status
- MAVLink connection monitoring
- 800 × 480 LCD with 10 Hz display updates

## Hardware

- ESP32-S3 (16 MB Flash / 8 MB PSRAM)
- Guition JC8048W550, 5-inch RGB LCD
- CUAV V5 Nano Flight Controller
- Wi-Fi Network

## Development Tools

- VS Code + PlatformIO
- Arduino Framework / C++
- Arduino_GFX v1.5.9
- MAVLink C Library
- WiFi / WiFiUDP

## Communication

`ArduPilot → Mission Planner → Wi-Fi UDP → ESP32-S3 → LCD`

UDP Port: `14550`

## Getting Started

See [Setup Guide](docs/SETUP.md) for installation, configuration, and testing instructions.

## License

MIT License — Copyright © 2026 LEADING EDGE UNMANNED SYSTEMS.

*Experimental development project. Not intended for use as certified flight instrumentation.*

**LEUS — Flight • Autonomy • Innovation**
