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

float lastscantime;
long duration;
int distance;

void setup() {
    Serial.begin(115200);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    pinMode(ledGPin, OUTPUT);
    pinMode(ledRPin, OUTPUT);
    ledcAttach(ledRPin, freq, res);
    ledcAttach(ledGPin, freq, res);
}

void loop() {
  distancecheck();
}

void distancecheck(){

  if(lastscantime - millis() <= 200){
    //triggering scan
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
      
    duration = pulseIn(echo, HIGH, 50000); // Timeout of 50ms
    // calculating distance
    distance = duration * 0.034 / 2;
    lastscantime = millis();
  // Calculate PWM value based on distance, use floating-point division
    float pwm = 355 * (1.0 / distance); // 355 used for higher voltage to led - better visability, less strobing due to distance change
    pwm = constrain(pwm, 0, 255);  // Clamp PWM to 0-255 range

    // Adjusting LED brightness based on distance
    if (distance <= 10) {
        ledcWrite(ledRPin, pwm);  // Use the correct channel for the red LED
    } else {
        ledcWrite(ledRPin, LOW);  // Turn off the red LED
    }
    
    // Debugging output
    Serial.print("Distance: ");
    Serial.println(distance);
  }

}
