# LEUS-ESP32-MAVLink-HUD
An Experimental Flight Display for UAV Telemetry
Developed by LEADING EDGE UNMANNED SYSTEMS (LEUS)
Overview
LEUS ESP32-S3 MAVLink HUD is an open-source experimental flight display developed using an ESP32-S3 microcontroller and a 5-inch RGB LCD.
The project demonstrates how MAVLink telemetry data from an ArduPilot-based flight controller can be received over Wi-Fi UDP and displayed in real time.
The objective is to explore embedded graphics, UAV telemetry communication, and lightweight flight information displays using affordable development hardware.
Features
-Real-time MAVLink telemetry reception via Wi-Fi UDP
-Artificial Horizon with Roll and Pitch indication
-Pitch Ladder and Roll Angle Scale
-Roll, Pitch, and Yaw numerical display
-Ground Speed and Relative Altitude
-Battery Voltage and GPS Fix Status
-Flight Mode and Armed/Disarmed indication
-MAVLink connection status
-800 × 480 RGB LCD interface
-10 Hz display update rate, tested in the development setup
Hardware Requirements

Component
-Microcontroller: ESP32-S3, 16 MB Flash, 8 MB PSRAM
-Display:Guition JC8048W550, 5-inch RGB LCD, 800 × 480
-Flight Controller:CUAV V5 Nano
-Firmware: ArduCopter / ArduPilot
-Communication:Wi-Fi UDP
-Ground Control Station:Mission Planner
-Development Tools and Libraries:Visual Studio Code
-PlatformIO:Arduino Framework for ESP32
-C/C++
-Arduino_GFX Library v1.5.9
-MAVLink C Library 
-ESP32 WiFi and WiFiUDP Libraries
[The tested development environment uses Espressif32 PlatformIO 7.0.1 and Arduino ESP32 Core 2.0.17.]
