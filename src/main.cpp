#include <Arduino.h>
#include "servoTof.h"
#include "motor.h"
#include "motionControl.h"

// --- USER-CONFIGURABLE PARAMETERS ---
const int SERVO_PIN = 13;                        // Servo control pin
const int STEP_ANGLE = 5;                        // Servo movement step in degrees
const uint16_t DETECTION_THRESHOLD_MM = 150;     // Object detection distance threshold (mm)
const unsigned long ROTATION_DURATION_MS = 2000; // How long to rotate (ms)
const unsigned long SERVO_INTERVAL_MS = 50;      // How often to move the servo (ms)

// --- TIMING STATE ---
unsigned long lastServoMoveTime = 0;
unsigned long rotationStartTime = 0;
bool rotationStarted = false;

// --- ROBOT STATE MACHINE ---
enum RobotState
{
  SEARCHING_FORWARD,
  FIRST_DETECTION
};
RobotState currentState = SEARCHING_FORWARD;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
  }

  initServo(SERVO_PIN);
  initTof();
  initMotors();

  setMotionMode(MOVE_FORWARD);
  updateMotors();
}

void loop()
{
  unsigned long now = millis();

  switch (currentState)
  {
  case SEARCHING_FORWARD:
    // Ensure car is moving forward
    if (currentMode != MOVE_FORWARD)
    {
      setMotionMode(MOVE_FORWARD);
      updateMotors();
    }

    // Move servo every SERVO_INTERVAL_MS
    if (now - lastServoMoveTime >= SERVO_INTERVAL_MS)
    {
      lastServoMoveTime = now;

      updateServoDirection(STEP_ANGLE);
      writeServoAngle();

      uint16_t dist = readDistance();

      Serial.print(getCurrentAngle());
      Serial.print("\t");
      Serial.println(dist);

      if (objectDetected(dist, DETECTION_THRESHOLD_MM))
      {
        currentState = FIRST_DETECTION;
      }
    }
    break;

  case FIRST_DETECTION:
    if (!rotationStarted)
    {
      Serial.print("Object detected at angle: ");
      Serial.println(getCurrentAngle());

      // Stop the car immediately
      setMotionMode(STOP);
      updateMotors();

      // Reset servo to center
      resetServo();

      rotationStartTime = now; // Start rotation timing
      rotationStarted = true;  // Mark that rotation has started

      // Start rotating (simulate)
      setMotionMode(ROTATE_LEFT);
      updateMotors();

      currentState = SEARCHING_FORWARD;
    }
    else
    {
      // Wait for rotation duration, then resume moving forward
      if (now - rotationStartTime >= ROTATION_DURATION_MS)
      {
        Serial.println("Rotation done, resuming forward movement...");

        setMotionMode(MOVE_FORWARD);
        updateMotors();

        currentState = SEARCHING_FORWARD;
        lastServoMoveTime = now - SERVO_INTERVAL_MS; // Force servo to move immediately
        rotationStarted = false; // Rotation finished, reset flag for next detection
      }
      break;
    }
  }
}