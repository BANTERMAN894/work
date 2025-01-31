#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define I2C_SLAVE_ADDR  0x08

//  MPU6050 setup
Adafruit_MPU6050 mpu;
float xoffset = 0.52;
float yoffset = -0.16;
float accX;
float accY;

//  Ultrasonic Sensor
const int trig = 23;
const int echo = 34;

//  LEDs
const int ledGPin = 13;
const int ledRPin = 15;
const int freq = 5000;
const int res = 8;

const int ledRChannel = 0;
const int ledGChannel = 1;

//  IR Sensor

const int numSensors = 6; // Number of sensors
const int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5}; // Analog pins for sensors
float sensorValues[6]; // stores sensor values

//  Motion constants

int Rspeed;
int Lspeed;

// Slave

int leftRPM;
int rightRPM;

//  Setting variables

float lastacceltime;
float lastscantime;
long duration;
int distance;

void setup() {

  //  Setting up serial monitor + I2C

    Serial.begin(115200);
    Wire.begin();

  //  Setup Ultrasonic

    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
  
  //  Setup LED 

    pinMode(ledGPin, OUTPUT);
    pinMode(ledRPin, OUTPUT);
    ledcAttach(ledRPin, freq, res);
    ledcAttach(ledGPin, freq, res);

  //  Setup MPU

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    sendFunction(1,Rspeed,Lspeed,90);

}

void loop() {

  //  Checking for new data......
  requestRPM();
  distancecheck();
  accelcheck();

  //  Displaying new data

  Serial.Println("Left Speed: ");
  Serial.Println(leftRPM);
  Serial.Println("Right Speed: ");
  Serial.Println(rightRPM);
  Serial.Println("Distance: ");
  Serial.Println(distance);
  Serial.Println("X Accel: ");
  Serial.Println(accX);
  Serial.Println("Y Accel: ");
  Serial.Println(accY);

}



//-------------------------Functions---------------------------------



//  Ultrasonic sensor

void distancecheck(){

//  Timer, checks every 200ms

  if(lastscantime - millis() <= 200){

  //  Triggering scan
    
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
  
  //  Wait 50ms for pulse, Max distance 850cm, keeps consistant results

    duration = pulseIn(echo, HIGH, 50000); 

  //  Calculating distance

    distance = duration * 0.034 / 2;
    lastscantime = millis();

  //  Calculate PWM value based on distance, use floating-point division
  // 355 used for higher voltage to led - better visability, less strobing due to distance/pwm delta
    float pwm = 355 * (1.0 / distance); 

  // Clamp PWM to 0-255 range

    pwm = constrain(pwm, 0, 255);  

  //  Adjusting LED brightness based on distance

    if (distance <= 10) {

    //  Setting currrent LED brightness

        ledcWrite(ledRPin, pwm);  
      
    } else {

    //  Turning off LED

        ledcWrite(ledRPin, LOW);

    }
    
  }

}


//  MPU6050 

void accelcheck(){

//  Timer, checks every 100ms

  if(lastacceltime - millis() <= 100){

  //  Get new sensor events with the readings 

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

  //  Save values

    accX = a.acceleration.x - xoffset;
    accY = a.acceleration.y - yoffset;

  //  Reset timer

    lastacceltime = millis();

  }
}


// IR Sensor

void linecheck(){

  for (int i = 0; i < numSensors, i++){

    sensorValues[i] = digitalRead(sensorPins[i]);
    Serial.println(sensorValues[i]);

  }

  if (sensorValues = [1]*6){

    sendFunction(5,0,0,90);

  }
  if(sensorValues[3,4] = [1,1]){

    sendFunction(1,Rspeed,Lspeed,90);

  }
}


//  I2C 

//  Motion notes 

//  Angle = 90 @ straight

//  Command motions 1 - Forward 2 - Backward 3 - Left 4 - Right 5 - Stop

void sendFunction(uint8_t command, int leftSpeed, int rightSpeed, int Angle) {
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    
    // Send parameters (each int is 2 bytes)
    Wire.write(leftSpeed >> 8);  // High byte of left speed
    Wire.write(leftSpeed & 0xFF); // Low byte of left speed

    Wire.write(rightSpeed >> 8);  // High byte of right speed
    Wire.write(rightSpeed & 0xFF); // Low byte of right speed

    Wire.write(Angle >> 8);  // High byte of Angle
    Wire.write(Angle & 0xFF); // Low byte of Angle

    // Send the command byte
    Wire.write(command);

    Wire.endTransmission();
}


// Function to request motor RPM from the slave
void requestRPM() {
    Wire.requestFrom(I2C_SLAVE_ADDR, 4); // Request 4 bytes (2x 16-bit integers)

    if (Wire.available() == 4) {
        leftRPM = (Wire.read() << 8) | Wire.read();  // Read left motor RPM
        rightRPM = (Wire.read() << 8) | Wire.read(); // Read right motor RPM
        
        Serial.print("Left Motor RPM: ");
        Serial.print(leftRPM);
        Serial.print(" | Right Motor RPM: ");
        Serial.println(rightRPM);
    }
}
