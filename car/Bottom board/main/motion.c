#include "motion.h"
#include <ESP32Servo.h>
#include <Wire.h>

void homeServo() {
    // Move to the minimum position
    myServo.write(minPosition);
    delay(1000); // Wait for the servo to reach the position
    Serial.println("Homed to minimum position.");
}

void moveToMiddle() {
    // Move to the middle position
    myServo.write(middlePosition);
    delay(1000); // Wait for the servo to reach the position
    Serial.println("Moved to middle position.");
}

void motors(int leftSpeed, int rightSpeed) {
  // set individual motor speed
  // the direction is set separately

  // constrain the values to within the allowable range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  ledcWrite(enA, leftSpeed);
  ledcWrite(enB, rightSpeed);
}

void goForwards(float Movetime) {
  moveStartTime = millis();
  moveDuration = Movetime;
  ismoving = true;
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}


void goBackwards(float Movetime) {
  moveStartTime = millis();
  moveDuration = Movetime;
  ismoving = true;
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void goClockwise(float Movetime) {
  
}

void goAntiClockwise(float Movetime) {
  
}

// Function to stop the motors
void stopMotors() {
    digitalWrite(INa, LOW);
    digitalWrite(INb, LOW);
    digitalWrite(INc, LOW);
    digitalWrite(INd, LOW);
    Serial.println("Motors stopped");
}

void countLeftMotorR() {
    leftMotorR++;
}

void countRightMotorR() {
    rightMotorR++;
}

void updateRPM() {
    unsigned long currentTime = millis();
    unsigned long timeElapsed = currentTime - lastRPMUpdateTime;

    if (timeElapsed >= 1000) {  // Update every second
        leftMotorRPM = (leftMotorR / (float)enc_poles) * 60.0;  // Convert to RPM
        rightMotorRPM = (rightMotorR / (float)enc_poles) * 60.0;

        leftMotorR = 0;  // Reset pulse counts
        rightMotorR = 0;
        lastRPMUpdateTime = currentTime;

        Serial.print("Left Motor RPM: ");
        Serial.println(leftMotorRPM);
        Serial.print("Right Motor RPM: ");
        Serial.println(rightMotorRPM);
    }
}