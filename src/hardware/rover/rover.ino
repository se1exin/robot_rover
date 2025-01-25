#include <AccelStepper.h>

// Pin Definitions for Motors
// FRONT LEFT - PORT E
const int MOTOR1_STEP_PIN = 23;
const int MOTOR1_DIR_PIN = 31;

// FRONT RIGHT - PORT A
const int MOTOR2_STEP_PIN = 28;
const int MOTOR2_DIR_PIN = 36;

// Motor Interface Type
#define MOTOR_INTERFACE_TYPE AccelStepper::DRIVER

// Create motor objects
AccelStepper motor1(MOTOR_INTERFACE_TYPE, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(MOTOR_INTERFACE_TYPE, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);

// Speed limits
const int maxSpeed = 100;      // Maximum speed in steps per second
const int acceleration = 0;     // Acceleration in steps per second squared

// Timeout variables
unsigned long lastCommandTime = 0;           // Time of the last received command
const unsigned long commandTimeout = 5000;   // Timeout in milliseconds (5 seconds)

void setup() {
  // Initialize serial communication
  Serial.begin(115200); // Use a higher baud rate for better performance

  // Set maximum speed and acceleration for each motor
  motor1.setMaxSpeed(maxSpeed);
  motor1.setAcceleration(acceleration);

  motor2.setMaxSpeed(maxSpeed);
  motor2.setAcceleration(acceleration);

  Serial.println("Robot ready for real-time control...");
}

void loop() {
   Serial.println("Robot ready for real-time control...");
  // Check for incoming serial commands asynchronously
  static String commandBuffer = ""; // Buffer to store incoming serial data
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == '\n') {
      // Complete command received, process it
      commandBuffer.trim(); // Remove any trailing spaces or newlines
      parseCommand(commandBuffer);
      commandBuffer = ""; // Clear the buffer for the next command
      lastCommandTime = millis(); // Reset the timeout timer
    } else {
      // Append character to the buffer
      commandBuffer += receivedChar;
    }
  }

  // Check for timeout and stop motors if needed
  if (millis() - lastCommandTime > commandTimeout) {
    stopMotors();
  }

  // Continuously update motor positions
  motor1.runSpeed();
  motor2.runSpeed();
}

// Function to parse and execute commands
void parseCommand(String command) {
  Serial.println("THING");
  // Convert String to a C-style character array
  char commandArray[50];
  command.toCharArray(commandArray, 50);

  // Parse the command using strtok
  char *token = strtok(commandArray, ",");
  int speeds[2];
  int idx = 0;

  while (token != nullptr && idx < 2) {  // Only handle two motors
    speeds[idx] = atoi(token); // Convert token to integer
    token = strtok(nullptr, ",");
    idx++;
  }

  if (idx == 2) {
    // Set motor speeds
    motor1.setSpeed(speeds[0]); // Motor 1
    motor2.setSpeed(speeds[1]); // Motor 2
  } else {
    Serial.println("Invalid command format");
  }
}

// Function to stop all motors
void stopMotors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}
