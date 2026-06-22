STM32 Engine Control Unit (ECU)

An educational Engine Control Unit (ECU) prototype developed using the STM32 Nucleo-F446RE microcontroller. This project demonstrates the basic principles of electronic engine management, including throttle position sensing, RPM estimation, fuel injection control, ignition control, PWM motor control, and serial telemetry.

Features
Throttle Position Sensor (TPS) simulation using a potentiometer
Engine RPM calculation
Fuel injection event simulation
Ignition event simulation
PWM-based motor speed control
Real-time UART telemetry output
Bare-metal STM32 firmware development
Hardware Used
STM32 Nucleo-F446RE
Potentiometer (Throttle Position Sensor)
L298N Motor Driver
DC Motor
LEDs for Injection and Ignition indication
USB Serial Connection
Pin Configuration
Function	Pin
TPS Input (ADC)	PA0
UART TX	PA2
UART RX	PA3
Injection LED	PB3
Ignition LED	PB4
Motor IN1	PB5
Motor IN2	PA8
PWM Output	PA15
System Overview

The potentiometer acts as a throttle position sensor. The STM32 reads the throttle value using its ADC and adjusts motor speed through PWM. Based on throttle position, the firmware estimates engine RPM and generates simulated ignition and fuel injection events. System data is continuously transmitted through UART for monitoring.

Project Structure
STM32-Engine-Control-Unit/
│
├── README.md
├── Firmware/
│   ├── Core/
│   ├── Drivers/
│
├── Docs/
│   ├── Block_Diagram.png
│   ├── Wiring_Diagram.png
│
├── Images/
│   ├── Setup.jpg
│   
│
└── LICENSE
UART Telemetry Example
TPS: 45%
RPM: 1850
Injection: ON
Ignition: ON
Motor PWM: 45%
--------------------
Learning Objectives

This project was developed to gain hands-on experience with:

Embedded C Programming
STM32 Microcontrollers
ADC
PWM Generation
UART Communication
Bare-Metal Register Programming
Real-Time Embedded Systems
Future Improvements
Hall Effect Sensor based RPM measurement
Closed-loop RPM control
CAN Bus communication
Knock detection simulation
Fuel and ignition mapping tables
FreeRTOS integration
Disclaimer

This project is intended for educational and demonstration purposes only. It does not control a real internal combustion engine and should not be used in safety-critical applications.

Author

Aditya Naigaonkar
Instrumentation and Control Engineering
VIT Pune
