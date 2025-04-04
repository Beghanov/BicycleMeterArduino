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
