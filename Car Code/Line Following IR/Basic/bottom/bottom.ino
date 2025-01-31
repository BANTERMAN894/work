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

//encoder setup and RPM variables
long leftMotorRPM = 0;
float rightMotorRPM = 0;
volatile unsigned long enc1_lastTime = 0, enc2_lastTime = 0;
volatile unsigned long enc1_timePerPulse = 0, enc2_timePerPulse = 0;

// Servo setup
Servo myServo;
const int servoPin = 13;

// Current servo position
int Angle;

// Set the speed of the motors
int leftSpeed = 240;
int rightSpeed = 240;

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

  Wire.begin(0x04);               //  I2c Setup
  Wire.onReceive(receiveEvent);
  Wire.onRequest(sendRPM);

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
  
}

void loop() {

  myServo.write(Angle);
  motors(leftSpeed, rightSpeed);
  delay(25); // allowing time for interrupts from onrequest and recieve

}

//-------------------------Functions---------------------------------

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
void goForwards() {

    digitalWrite(INa, LOW);
    digitalWrite(INb, HIGH);
    digitalWrite(INc, LOW);
    digitalWrite(INd, HIGH);

}

//  Moves Backwards
void goBackwards() {

    digitalWrite(INa, HIGH);
    digitalWrite(INb, LOW);
    digitalWrite(INc, HIGH);
    digitalWrite(INd, LOW);

}

//  Motors assist in turning
void turnLeft() {

    digitalWrite(INa, LOW);
    digitalWrite(INb, HIGH);
    digitalWrite(INc, HIGH);
    digitalWrite(INd, LOW);

}

//  Motor's assist in turning
void turnRight() {

    digitalWrite(INa, HIGH);
    digitalWrite(INb, LOW);
    digitalWrite(INc, LOW);
    digitalWrite(INd, HIGH);

}



//  I2C Bus comms

//  Function to clear the I2C buffer
void emptyBuffer(void) {

  Serial.println("Error: I2C Byte Size Mismatch");
  while (Wire.available()) {
  Wire.read();

  }
}

//  Function to send left and right RPM to the master
void sendRPM() {
    uint16_t leftRPM = (uint16_t)leftMotorRPM;
    uint16_t rightRPM = (uint16_t)rightMotorRPM;

    Wire.write(leftRPM >> 8);     // High byte of left RPM
    Wire.write(leftRPM & 0xFF);   // Low byte of left RPM
    Wire.write(rightRPM >> 8);    // High byte of right RPM
    Wire.write(rightRPM & 0xFF);  // Low byte of right RPM
}

//  Handles function requests from master ESP for motion

void receiveEvent(int bytes) {
  // Ensure at least 5 bytes received (1 command + 2 ints)
    if (bytes < 7) return;  

    if (Wire.available()) {
        leftSpeed = (Wire.read() << 8) | Wire.read();  // Read first int (2 bytes)
        rightSpeed = (Wire.read() << 8) | Wire.read();  // Read second int (2 bytes)
        Angle = (Wire.read() << 8) | Wire.read();  // Read third int (2 bytes)
        uint8_t command = Wire.read();
        Serial.print("Received Command: ");
        Serial.println(command);

  // Execute a function based on command received
        switch (command) {
          case 1:
            goForwards();
            break;
          case 2:
            goBackwards();
            break;
          case 3:
            turnLeft();
            break;
          case 4:
            turnRight();
            break;
          case 5:
            stopMotors();
            break;
          default:
            break;
        }
    }
}