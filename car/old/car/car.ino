#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>

// Wifi settings
const char* ssid = "Rlap";
const char* password = "abc12345";

// Create a web server on port 80
WebServer server(80);

#define enA 33  // enableA command line
#define enB 25  // enableB command line

#define INa 26  // channel A direction
#define INb 27  // channel A direction
#define INc 14  // channel B direction
#define INd 12  // channel B direction

// is moving
bool ismoving = false;
unsigned long moveStartTime = 0;
float moveDuration = 0;

//current servo pos
int currentServoPosition = 90;

// Set the speed of the motors
int leftSpeed = 240;
int rightSpeed = 230;

// Define the servo pin
const int servoPin = 13; // Change this to your desired pin
Servo myServo;

// Define the servo positions
const int minPosition = 0;     // Minimum position (0 degrees)
const int maxPosition = 180;   // Maximum position (180 degrees)
const int middlePosition = 90;  // Middle position (90 degrees)

// setting PWM properties
const int freq = 2000;
const int ledChannela = 1;  // the ESP32 servo library uses the PWM channel 0 by default, hence the motor channels start from 1
const int ledChannelb = 2;
const int resolution = 8;

void setup() {
    Serial.begin(9600);
    delay(500);
    Serial.println("ESP32 Running");

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Define routes for the buttons
    server.on("/", HTTP_GET, handleRoot);
    server.on("/button1", HTTP_GET, handleButton1);
    server.on("/button2", HTTP_GET, handleButton2);
    server.on("/setValue", HTTP_GET, handleSetValue);

    server.begin();

    // configure the LED PWM functionalitites and attach the GPIO to be controlled - ensure that this is done before the servo channel is attached
    ledcAttachChannel(enA, freq, resolution, ledChannela);
    ledcAttachChannel(enB, freq, resolution, ledChannelb);

    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    myServo.attach(servoPin);

    // Left Motor
    pinMode(INa, OUTPUT);
    pinMode(INb, OUTPUT);
    // Right Motor
    pinMode(INc, OUTPUT);
    pinMode(INd, OUTPUT);

    // Home the servo to the minimum position
    homeServo();
    delay(2000);
    // Move to the middle position
    moveToMiddle();
    ismoving = false;
    goForwards(2000);
}

void loop() {
    // Check if we are currently moving
    if (ismoving == true) {
        // Check if the move duration has passed
        if ((millis() - moveStartTime) >= moveDuration) {
            stopMotors();
            ismoving = false; // Stop moving
            moveDuration = 0;
            Serial.println("Stopped moving");
        }
    }
}

void handleRoot() {
    String html = "<!DOCTYPE HTML><html><head><title>ESP32 Control</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body { text-align: center; } button { font-size: 20px; margin: 10px; }</style></head>";
    html += "<body><h2>Control ESP32</h2>";
    html += "<button id='buttonForward' onclick=\"sendRequest('/button1')\">Move Forward</button>";
    html += "<button id='buttonBack' onclick=\"sendRequest('/button2')\">Move Back</button>";
    html += "<label for='slider'>Adjust Variable:</label>";
    html += "<input type='range' id='slider' min='60' max='120' value='90' oninput='updateValue(this.value)'>";
    html += "<p>Current Value: <span id='sliderValue'>50</span></p>";
    html += "<script>function sendRequest(button) { var xhr = new XMLHttpRequest(); xhr.open('GET', button, true); xhr.send(); }";
    html += "function updateValue(value) { document.getElementById('sliderValue').innerText = value; var xhr = new XMLHttpRequest(); xhr.open('GET', '/setValue?value=' + value, true); xhr.send(); }";
    html += "document.addEventListener('keydown', function(event) {";
    html += "var slider = document.getElementById('slider');";
    html += "if (event.key === 'ArrowUp') { document.getElementById('buttonForward').click(); }";
    html += "else if (event.key === 'ArrowDown') { document.getElementById('buttonBack').click(); }";
    html += "else if (event.key === 'ArrowLeft') {";
    html += "var newValue = Math.max(parseInt(slider.value) - 5, slider.min);"; // Decrease slider value
    html += "slider.value = newValue;";
    html += "updateValue(newValue);"; // Update the displayed value
    html += "} else if (event.key === 'ArrowRight') {";
    html += "var newValue = Math.min(parseInt(slider.value) + 5, slider.max);"; // Increase slider value
    html += "slider.value = newValue;";
    html += "updateValue(newValue);"; // Update the displayed value
    html += "}});</script>";
    html += "</body></html>";

    server.send(200, "text/html", html);
}

// Function for Button 1
void handleButton1() {
  // Stop any ongoing motor movement before executing the new command
  stopMotors();
  // Activate function for Button 1
  Serial.println("Moving Forward");
  motors(leftSpeed, rightSpeed);
  goForwards(2000);

}

// Function for Button 2
void handleButton2() {
// Stop any ongoing motor movement before executing the new command
  stopMotors();
  // Activate function for Button 1
  Serial.println("Moving Forward");
  motors(leftSpeed, rightSpeed);
  goBackwards(500);  
}

void handleSetValue() {
    if (server.hasArg("value")) {
        int value = server.arg("value").toInt(); // Get the value from the query parameter
        myServo.write(value);
    }
    server.send(200, "text/plain", "Value updated");
}

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

void motors(int leftSpeed, int rightSpeed) {
  // set individual motor speed
  // the direction is set separately

  // constrain the values to within the allowable range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  ledcWrite(enA, leftSpeed);
  ledcWrite(enB, rightSpeed);
}

void goForwards(float Movetime) {
  moveStartTime = millis();
  moveDuration = Movetime;
  ismoving = true;
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}


void goBackwards(float Movetime) {
  moveStartTime = millis();
  moveDuration = Movetime;
  ismoving = true;
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void goClockwise(float Movetime) {
  
}

void goAntiClockwise(float Movetime) {
  
}

// Function to stop the motors
void stopMotors() {
    digitalWrite(INa, LOW);
    digitalWrite(INb, LOW);
    digitalWrite(INc, LOW);
    digitalWrite(INd, LOW);
    Serial.println("Motors stopped");
}