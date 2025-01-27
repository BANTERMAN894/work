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

#define ENC1_A_PIN 36  // GPIO for Encoder 1 Signal A
#define ENC1_B_PIN 39  // GPIO for Encoder 1 Signal B
#define ENC2_A_PIN 34  // GPIO for Encoder 2 Signal A
#define ENC2_B_PIN 35  // GPIO for Encoder 2 Signal B

// Global variable definitions
Servo myServo;
bool ismoving = false;
unsigned long moveStartTime = 0;
float moveDuration = 0;

//encoder setup and RPM variables
long leftMotorRPM = 0;
float rightMotorRPM = 0;
volatile unsigned long enc1_lastTime = 0, enc2_lastTime = 0;
volatile unsigned long enc1_timePerPulse = 0, enc2_timePerPulse = 0;

// Servo setup
const int servoPin = 13; // Change this to your desired pin

// Current servo position
int currentServoPosition = 90;

// Set the speed of the motors
int leftSpeed = 240;
int rightSpeed = 240;

// Define the servo positions
const int minPosition = 0;     // Minimum position (0 degrees)
const int maxPosition = 180;   // Maximum position (180 degrees)
const int middlePosition = 90;  // Middle position (90 degrees)

// Setting PWM properties
const int freq = 2000;
const int ledChannela = 1;  // the ESP32 servo library uses the PWM channel 0 by default, hence the motor channels start from 1
const int ledChannelb = 2;
const int resolution = 8;

void IRAM_ATTR handleEncoder1A() {
  unsigned long currentTime = micros();
  enc1_timePerPulse = currentTime - enc1_lastTime;
  enc1_lastTime = currentTime;
  leftMotorRPM = (60.0 * 1000000.0) / (enc1_timePerPulse * 12);
}

void IRAM_ATTR handleEncoder2A() {
  unsigned long currentTime = micros();
  enc2_timePerPulse = currentTime - enc2_lastTime;
  enc2_lastTime = currentTime;
  rightMotorRPM = (60.0 * 1000000.0) / (enc2_timePerPulse * 12);
}

void setup() {

  Serial.begin(9600);
  Serial.println("ESP32 Running");
  //Setup Servo
    myServo.attach(servoPin);
  // Setup PWM for motors
    ledcAttachChannel(enA, freq, resolution,15);
    ledcAttachChannel(enB, freq, resolution,14);
  // Attaching Encoders

  // Encoder 1 setup
  pinMode(ENC1_A_PIN, INPUT_PULLUP);
  pinMode(ENC1_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A_PIN), handleEncoder1A, RISING);

  // Encoder 2 setup
  pinMode(ENC2_A_PIN, INPUT_PULLUP);
  pinMode(ENC2_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC2_A_PIN), handleEncoder2A, RISING);

  // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
  // Left Motor
    pinMode(INa, OUTPUT);
    pinMode(INb, OUTPUT);
  // Right Motor
    pinMode(INc, OUTPUT);
    pinMode(INd, OUTPUT);

  // Home the servo to the minimum position
    homeServo();
    delay(1000);
  // Move to the middle position
    moveToMiddle();
    ismoving = false;
    goForwards(1000);

}

void loop() {

  // Check if we are currently moving
  if (ismoving == true) {        // Check if the move duration has passed

    if ((millis() - moveStartTime) >= moveDuration) {

      stopMotors();
      ismoving = false; // Stop moving
      moveDuration = 0;
      leftMotorRPM = 0;
      rightMotorRPM = 0;
      Serial.println("Stopped moving");
    }
  }
}

//-------------------------Functions---------------------------------

// Servo Functions

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


//  Motion

//  Sets motor speeds
void motors(int leftSpeed, int rightSpeed) {

  // set individual motor speed
  // the direction is set separately

  // constrain the values to within the allowable range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  ledcWrite(enA, leftSpeed);
  ledcWrite(enB, rightSpeed);

}

// Function to stop the motors

//  Stops Motors
void stopMotors() {

    motors(0,0);
    digitalWrite(INa, LOW);
    digitalWrite(INb, LOW);
    digitalWrite(INc, LOW);
    digitalWrite(INd, LOW);
    Serial.println("Motors stopped");

}

//  Moves Forwards
void goForwards(float Movetime) {
    motors(leftSpeed, rightSpeed);
    moveStartTime = millis();
    moveDuration = Movetime;
    ismoving = true;
    digitalWrite(INa, LOW);
    digitalWrite(INb, HIGH);
    digitalWrite(INc, LOW);
    digitalWrite(INd, HIGH);

}

//  Moves Backwards
void goBackwards(float Movetime) {

    moveStartTime = millis();
    moveDuration = Movetime;
    ismoving = true;
    digitalWrite(INa, HIGH);
    digitalWrite(INb, LOW);
    digitalWrite(INc, HIGH);
    digitalWrite(INd, LOW);

}


// I2C Bus comms


//Handles requests from master ESP
void onRequestHandler() {

  Wire.write(ismoving ? 1 : 0); // Moving status
  // Send left and right motor RPM as 16-bit values
  unsigned int leftRPM = (unsigned int)leftMotorRPM; // Changed to unsigned int
  int rightRPM = (int)rightMotorRPM; // Changed to int
  Wire.write(leftRPM >> 8);    // High byte of left motor RPM
  Wire.write(leftRPM & 0xFF); // Low byte of left motor RPM
  Wire.write(rightRPM >> 8);  // High byte of right motor RPM
  Wire.write(rightRPM & 0xFF); // Low byte of right motor RPM

}


//Function to clear the I2C buffer
void emptyBuffer(void) {

  Serial.println("Error: I2C Byte Size Mismatch");
  while (Wire.available()) {
  Wire.read();

  }
}