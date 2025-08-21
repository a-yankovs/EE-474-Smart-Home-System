# LiteWork – Multi-Node Smart Light System
Version: 1.3

LiteWork is a multi-node smart lighting system built on ESP32 microcontrollers.
It intelligently controls lights based on ultrasonic motion sensing, BLE app commands, a physical button, and ESP-NOW messages exchanged between nodes (e.g., between multiple rooms).

## Features

Multi-source light control

Ultrasonic motion detection for automatic entry/exit recognition.

BLE mobile app control for ON/OFF commands, time, and alarm scheduling.

Physical button interrupt to toggle ultrasonic sensing mode.

ESP-NOW wireless messaging between nodes for synchronized room lighting.

### Servo-based light switch control

Two servos simulate pressing a physical switch to turn lights on/off.

Supports single toggles and "burst flashing" (party/alert mode).

LCD feedback

Displays light state, sensor status, and party/alarm messages.

### Alarm system via BLE

Set system time and alarms from your phone app.

At alarm time, lights burst flash 5 times.

### Notifications sent back over BLE.

### FreeRTOS Task Scheduling 

Separate tasks for ultrasonic sensing, light control, ESP-NOW send/receive, and alarms.

Queues (commandQueue, espNowTx, espNowRx) for event-driven coordination.

## Hardware Setup
Pinout
Component	 

Servo (ON)	            GPIO 4

Servo (OFF)	            GPIO 5

Ultrasonic Sensor TRIG	GPIO 17

Ultrasonic Sensor ECHO	GPIO 16

Button (ISR toggle)	    GPIO 2

I2C SDA (LCD)	          GPIO 8
  
I2C SCL (LCD)	          GPIO 9

LCD: I²C 16x2, default address 0x27.

Ultrasonic sensor: detects presence within ~50 cm.

Servos: physically toggle a switch.

Button: enables/disables ultrasonic sensing (“party mode”).

## Communication

BLE Service UUID: 76aba4a6-eb39-4a35-85b2-a6fbb20999cf

Characteristic UUID: 2cdd5742-0b1a-4758-8096-cbc516b21eac

### BLE Commands

ON / OFF → Toggle lights.

TIME HH:MM:SS → Set internal clock.

ALARM HH:MM → Set alarm.

ALARM CLEAR → Clear alarm.

### ESP-NOW

Uses Channel 6 for communication.

Supports inter-room messaging:

Room1 OFF → Room2 ON (exit).

Room2 OFF → Room1 ON (entry).

## Software Structure

Light Control Task – Processes commands from BLE, ultrasonic, button, or ESP-NOW.

Ultrasonic Sense Task – Detects presence and triggers room entry/exit logic.

ESP-NOW Send/Receive Tasks – Handle inter-node wireless messages.

Alarm Timer – Fires alarms and notifies BLE app.

Button ISR – Toggles ultrasonic sensing mode.

## Setup & Installation

### Install Arduino IDE or PlatformIO.

Add ESP32 board support.

### Install required libraries:

ESP32Servo

LiquidCrystal_I2C

ESP-NOW (comes with ESP32)

Arduino BLE

Connect hardware per pinout above.

Flash the LiteWork.ino sketch.

## Usage

Normal Mode:
Lights turn ON/OFF based on ultrasonic detection and ESP-NOW room coordination.

BLE App Control:
Send ON or OFF to manually control lights.
Use TIME and ALARM commands for scheduling.

Party Mode:
Press the physical button → disables ultrasonic sensing, LCD displays PARTY TIME!!!, lights can be toggled manually.

Alarm:
At set alarm time, lights will burst flash and send BLE notification ALARM:FIRE.
