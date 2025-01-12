#include <AccelStepper.h>

/*
 * THOR BOARD PIN MAPPING: (STEP, DIR)
 * A 28, 36
 * B 26, 34
 * C 24, 32
 * D 22, 30
 * E 23, 31
 * F 25, 33
 * G 27, 35
 * H 
 */

// Pin Definitions for Motors
// FRONT LEFT - PORT E
const int MOTOR1_STEP_PIN = 23;
const int MOTOR1_DIR_PIN = 31;

// FRONT RIGHT - PORT A
const int MOTOR2_STEP_PIN = 28;
const int MOTOR2_DIR_PIN = 36;

// BACK LEFT - PORT F
const int MOTOR3_STEP_PIN = 27;
const int MOTOR3_DIR_PIN = 35;

// BACK RIGHT - PORT D
const int MOTOR4_STEP_PIN = 22;
const int MOTOR4_DIR_PIN = 30;

// Motor Interface Type
#define MOTOR_INTERFACE_TYPE AccelStepper::DRIVER

// Create motor objects
AccelStepper motor1(MOTOR_INTERFACE_TYPE, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(MOTOR_INTERFACE_TYPE, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
AccelStepper motor3(MOTOR_INTERFACE_TYPE, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN);
AccelStepper motor4(MOTOR_INTERFACE_TYPE, MOTOR4_STEP_PIN, MOTOR4_DIR_PIN);

// Speed limits
const int maxSpeed = 500;       // Maximum speed in steps per second
const int acceleration = 0;     // Acceleration in steps per second squared

// Timeout variables
unsigned long lastCommandTime = 0;           // Time of the last received command
const unsigned long commandTimeout = 5000;   // Timeout in milliseconds (1 second)

void setup() {
  // Initialize serial communication
  Serial.begin(115200); // Use a higher baud rate for better performance

  // Set maximum speed and acceleration for each motor
  motor1.setMaxSpeed(maxSpeed);
  motor1.setAcceleration(acceleration);

  motor2.setMaxSpeed(maxSpeed);
  motor2.setAcceleration(acceleration);

  motor3.setMaxSpeed(maxSpeed);
  motor3.setAcceleration(acceleration);

  motor4.setMaxSpeed(maxSpeed);
  motor4.setAcceleration(acceleration);

  Serial.println("Robot ready for real-time control...");
}

void loop() {
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
  motor3.runSpeed();
  motor4.runSpeed();
}

// Function to parse and execute commands
void parseCommand(String command) {
  // Convert String to a C-style character array
  char commandArray[50];
  command.toCharArray(commandArray, 50);

  // Parse the command using strtok
  char *token = strtok(commandArray, ",");
  int speeds[4];
  int idx = 0;

  while (token != nullptr && idx < 4) {
    speeds[idx] = atoi(token); // Convert token to integer
    token = strtok(nullptr, ",");
    idx++;
  }

  if (idx == 4) {
    // Set motor speeds
    motor1.setSpeed(speeds[0]); // Front Left Motor
    motor2.setSpeed(speeds[1]); // Front Right Motor
    motor3.setSpeed(speeds[2]); // Rear Left Motor
    motor4.setSpeed(speeds[3]); // Rear Right Motor
  } else {
    Serial.println("Invalid command format");
  }
}

// Function to stop all motors
void stopMotors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
}
