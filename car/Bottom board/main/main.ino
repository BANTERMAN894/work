#include <motion.h>
#include <handler.h>
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

#define I2C_SLAVE_ADDR 0x04 // I2C address

#define enA 33  // enableA command line
#define enB 25  // enableB command line

#define INa 26  // channel A direction
#define INb 27  // channel A direction
#define INc 14  // channel B direction
#define INd 12  // channel B direction

// Global variable definitions
Servo myServo;
bool ismoving = false;
float leftMotorRPM = 0;
float rightMotorRPM = 0;
int leftMotorR = 0;
int rightMotorR = 0;
unsigned long lastRPMUpdateTime = 0;
unsigned long moveStartTime = 0;
float moveDuration = 0;

// Encoder setup
const int enc_poles = 12;
const int encoderPinLeft = 3;   // Left motor encoder pin
const int encoderPinRight = 4;  // Right motor encoder pin

// Servo setup
const int servoPin = 13; // Change this to your desired pin

// Current servo position
int currentServoPosition = 90;

// Set the speed of the motors
int leftSpeed = 240;
int rightSpeed = 230;

// Define the servo positions
const int minPosition = 0;     // Minimum position (0 degrees)
const int maxPosition = 180;   // Maximum position (180 degrees)
const int middlePosition = 90;  // Middle position (90 degrees)

// Setting PWM properties
const int freq = 2000;
const int ledChannela = 1;  // the ESP32 servo library uses the PWM channel 0 by default, hence the motor channels start from 1
const int ledChannelb = 2;
const int resolution = 8;

void setup() {
    Serial.begin(9600);
    delay(500);
    Wire.begin(I2C_SLAVE_ADDR);   // Join I2C bus #4
    Wire.onRequest(onRequestHandler);  // Set up request handler
    Serial.println("ESP32 Running");

    // Encoder connections
    pinMode(encoderPinLeft, INPUT_PULLUP);
    pinMode(encoderPinRight, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPinLeft), countLeftMotorR, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinRight), countRightMotorR, RISING);
    
    // Setup PWM for motors
    ledcAttach(ledChannela, freq, resolution);
    ledcAttach(ledChannelb, freq, resolution);

    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    myServo.attach(servoPin);

    // Left Motor
    pinMode(INa, OUTPUT);
    pinMode(INb, OUTPUT);
    // Right Motor
    pinMode(INc, OUTPUT);
    pinMode(INd, OUTPUT);

    // Home the servo to the minimum position
    homeServo();
    delay(2000);
    // Move to the middle position
    moveToMiddle();
    ismoving = false;
    goForwards(2000);
}

void loop() {
    // Check if we are currently moving
    if (ismoving == true) {
        // Check if the move duration has passed
        if ((millis() - moveStartTime) >= moveDuration) {
            stopMotors();
            ismoving = false; // Stop moving
            moveDuration = 0;
            Serial.println("Stopped moving");
        }
    }

    // Call updateRPM to update the RPM values periodically
    updateRPM();
}