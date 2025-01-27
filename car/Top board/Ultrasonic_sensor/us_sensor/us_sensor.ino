#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
float AccX;
float AccY;
float AccZ;
float GyrX;
float GyrY;
float GyrZ;

Adafruit_MPU6050 mpu;

long duration;
int distance;

const int trig = 23;
const int echo = 34;
const int ledGPin = 13;
const int ledRPin = 15;
const int freq = 5000;
const int res = 8;

void setup(){
  pinMode(trig, OUTPUT);
  pinMode(ledGPin, OUTPUT);
  pinMode(ledRPin, OUTPUT);
  pinMode(echo, INPUT);
  Serial.begin(9600);
  ledcAttach(ledRPin, freq, res);
  ledcAttach(ledGPin, freq, res);
}

void loop(){
  // checking and updating HC SR04 distance
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  pinMode(echo, INPUT);
  duration = pulseIn(echo, HIGH);
  distance = duration * 0.034/2;
  if (distance <= 10){
    int cycle = (200);    
    ledcWrite(ledGPin, cycle * 1/distance);
  }
  else{
    digitalWrite(ledRPin, LOW);
  }

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  AccX = a.acceleration.x;
  AccY = a.acceleration.y;
  AccZ = a.acceleration.z;
  GyrX = g.gyro.x;
  GryY = g.gryo.y;
  GryZ = g.gyro.z;

}

