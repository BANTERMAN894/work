#include <Arduino.h>

// Ultrasonic Sensor
const int trig = 23;
const int echo = 34;
// LEDs
const int ledGPin = 13;
const int ledRPin = 15;
const int freq = 5000;
const int res = 8;

const int ledRChannel = 0;
const int ledGChannel = 1;

long duration;
int distance;

void setup() {
    Serial.begin(115200);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    pinMode(ledGPin, OUTPUT);
    pinMode(ledRPin, OUTPUT);
    ledcAttachPin(ledRPin, ledRChannel);
    ledcAttachPin(ledGPin, ledGChannel);
}

void loop() {
    distanceUpdate();
    // Debugging output
    Serial.print("Distance: ");
    Serial.println(distance);
  
}

void distanceUpdate(){
  // scan 10/s or every 100ms
  if(Lastscantime - millis() <=100){
  // Checking and updating HC-SR04 distance
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    duration = pulseIn(echo, HIGH, 50000); // Timeout of 50ms
    distance = duration * 0.034 / 2;
    Lastscantime = millis()
  // Adjusting LED brightness based on distance
    if (distance <= 10) {
      pwm = 255*(1/distance);
      ledcWrite(ledRPin, pwm);
    }
    else{
      ledcWrite(ledRPin, LOW);
    } 
  }
}
