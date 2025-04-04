# Bicycle Speedometer Project

## Description

This project is designed to measure the speed of a bicycle using an Arduino. It uses a **Hall sensor** to count wheel rotations and displays the current speed on an **LCD screen**. The speed is calculated in kilometers per hour (km/h) and updated every second.

## Components

- Arduino Uno
- Hall Sensor (or optical sensor)
- Magnet (to attach to the bike's wheel spoke)
- LCD 16x2 screen with I2C interface
- Wires

## Wiring

1. **Hall Sensor** is connected to pin 2 on the Arduino.
2. **Magnet** is attached to the bike's wheel spoke.
3. **LCD 16x2 screen** is connected via I2C. If using a regular screen, connect it directly to the Arduino pins (RS, EN, D4-D7).
4. Ensure the sensor and screen are connected properly, and Arduino is powered.

## Circuit Diagram

- **Hall Sensor:**
  - VCC -> 5V on Arduino
  - GND -> GND on Arduino
  - Signal -> Pin 2 on Arduino

- **LCD Screen:**
  - SDA -> A4 (for Arduino Uno)
  - SCL -> A5 (for Arduino Uno)
  - VCC -> 5V
  - GND -> GND

## Code

```cpp
#include <LiquidCrystal_I2C.h>

// Pins for connections
const int hallPin = 2;  // Hall sensor connected to pin 2
volatile int rotations = 0; // Number of rotations
float speed = 0;  // Speed in km/h

// Create LCD object
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address and screen size

// Variable for time interval
unsigned long lastTime = 0;
unsigned long interval = 1000;  // Update speed every 1 second

void setup() {
  pinMode(hallPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(hallPin), countRotation, RISING);  // Count rotations

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.setBacklight(1);
  lcd.print("Speed: 0 km/h");
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;
    
    // Calculate speed
    float wheelCircumference = 2.1;  // Wheel circumference in meters (measure your wheel)
    float distance = rotations * wheelCircumference;
    speed = (distance / 1000) * (3600 / (interval / 1000));  // Speed in km/h

    // Display speed on the screen
    lcd.clear();
    lcd.print("Speed: ");
    lcd.print(speed, 1);
    lcd.print(" km/h");

    rotations = 0;  // Reset rotations count
  }
}

void countRotation() {
  rotations++;
}
