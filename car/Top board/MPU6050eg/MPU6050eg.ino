// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
const float Xoffset = 0.51;
const float Yoffset = -0.16;
float AccX;
float AccY;
float turnangle;

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
}

void loop() {
   updateMPU();
}

void updateMPU(){
 /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  AccX = a.acceleration.x - Xoffset;
  AccY = a.acceleration.y - Yoffset;
  turnangle = atan2(AccY,AccX);
  Serial.print(AccX);
  Serial.println(AccY);
  Serial.print(turrnangle);
}
