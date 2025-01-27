#include "handler.h"
#include "motion.h"
#include <Wire.h>

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

// Function to clear the I2C buffer
void emptyBuffer(void) {
    Serial.println("Error: I2C Byte Size Mismatch");
    while (Wire.available()) {
        Wire.read();
    }
}