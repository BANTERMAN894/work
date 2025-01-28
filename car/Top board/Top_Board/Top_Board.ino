#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

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

//  Setting variables

float lastacceltime;
float lastscantime;
long duration;
int distance;

void setup() {

  //  Setting up serial monitor

    Serial.begin(115200);

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

}

void loop() {

  //  Checking for new data......

  distancecheck();
  accelcheck();

  //  Displaying new data

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