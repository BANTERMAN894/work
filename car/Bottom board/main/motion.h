#ifndef MOTION_H
#define MOTION_H

#include <ESP32Servo.h> // Include the correct Servo library for ESP32

// Global variables
extern Servo myServo;
extern bool ismoving;
extern float leftMotorRPM;
extern float rightMotorRPM;
extern int leftMotorR;
extern int rightMotorR;
extern unsigned long lastRPMUpdateTime;
extern unsigned long moveStartTime;
extern float moveDuration;

// Function declarations
void homeServo();
void moveToMiddle();
void motors(int leftSpeed, int rightSpeed);
void goForwards(float Movetime);
void goBackwards(float Movetime);
void goClockwise(float Movetime);
void goAntiClockwise(float Movetime);
void stopMotors();
void countLeftMotorR();
void countRightMotorR();
void updateRPM();

#endif
