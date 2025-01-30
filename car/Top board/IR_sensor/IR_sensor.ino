const int numSensors = 6; // Number of sensors
const int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5}; // Analog pins for sensors
const float weights[numSensors] = {1.0, 1.0, 1.5, 1.5, 1.0, 1.0}; // Weights for each sensor

void setup() {
  Serial.begin(9600); // Start serial communication

}

void loop() {
  // Read data from each sensor
  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    weightedSum += sensorValues[i] * weights[i]; // Calculate weighted sum
    weightSum += weights[i]; // Calculate sum of weights
  }

  // Calculate weighted average
  float weightedAverage = weightedSum / weightSum;

  // Print the weighted average
  Serial.print("Weighted Average: ");
  Serial.println(weightedAverage);

  // Reset variables for next iteration
  weightedSum = 0.0;
  weightSum = 0.0;

  delay(1000); // Delay to avoid overwhelming the serial monitor

}
