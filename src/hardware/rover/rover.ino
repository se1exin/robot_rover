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
 */

// Pin Definitions for Motors
// FRONT LEFT - PORT G
const int MOTOR1_STEP_PIN = 27;
const int MOTOR1_DIR_PIN = 31;

// FRONT RIGHT - PORT A
const int MOTOR2_STEP_PIN = 28;
const int MOTOR2_DIR_PIN = 36;

// BACK LEFT - PORT F
const int MOTOR3_STEP_PIN = 25;
const int MOTOR3_DIR_PIN = 33;

// BACK RIGHT - PORT B
const int MOTOR4_STEP_PIN = 26;
const int MOTOR4_DIR_PIN = 34;

// Motor Interface Type
#define MOTOR_INTERFACE_TYPE AccelStepper::DRIVER

// Create motor objects
AccelStepper motor1(MOTOR_INTERFACE_TYPE, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(MOTOR_INTERFACE_TYPE, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
AccelStepper motor3(MOTOR_INTERFACE_TYPE, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN);
AccelStepper motor4(MOTOR_INTERFACE_TYPE, MOTOR4_STEP_PIN, MOTOR4_DIR_PIN);

// Speed limits
const int maxSpeed = 200;       // Maximum speed in steps per second
const int acceleration = 1000;   // Acceleration in steps per second squared

// Watchdog timer
unsigned long lastCommandTime = 0; // Tracks the last time a command was received
const unsigned long commandTimeout = 2000; // Timeout period in milliseconds (2 seconds)

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

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
  // Check for incoming serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove trailing spaces or newlines

    // Parse the command
    parseCommand(command);

    // Update the watchdog timer
    lastCommandTime = millis();
  }

  // Stop the robot if no command is received within the timeout period
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
  // Command format: "SPEED_LF,SPEED_RF,SPEED_LR,SPEED_RR"
  // Example: "500,500,500,500" to move forward
  // Example: "-500,500,-500,500" to turn left
  if (command.indexOf(',') != -1) {
    int speeds[4]; // Array to store the speeds for each motor
    int idx = 0;
    while (command.length() > 0 && idx < 4) {
      int commaIndex = command.indexOf(',');
      if (commaIndex == -1) {
        speeds[idx] = command.toInt(); // Last value
        command = "";
      } else {
        speeds[idx] = command.substring(0, commaIndex).toInt();
        command = command.substring(commaIndex + 1);
      }
      idx++;
    }

    // Set motor speeds
    motor1.setSpeed(speeds[0]); // Front Left Motor
    motor2.setSpeed(speeds[1]); // Front Right Motor
    motor3.setSpeed(speeds[2]); // Rear Left Motor
    motor4.setSpeed(speeds[3]); // Rear Right Motor

//    Serial.println("Speeds set: " + String(speeds[0]) + ", " + String(speeds[1]) + ", " + String(speeds[2]) + ", " + String(speeds[3]));
  } else {
    Serial.println("Invalid command: " + command);
  }
}

// Function to stop all motors
void stopMotors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
  Serial.println("Motors stopped due to timeout.");
}
