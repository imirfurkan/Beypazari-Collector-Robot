#include <Arduino.h>
#include "motor.h"
#include "motionControl.h"
#include "ultrasonic.h"

// Constants
const int STBY = 30;
const unsigned int DIST_THRESHOLD = 80;    // cm
const unsigned int GRIPPER_THRESHOLD = 10; // cm
const int ROTATE_STEP_MS = 200;

// ──────────────────────────────
// Robot state machine (in main)
// ──────────────────────────────
enum RobotState
{
  SEARCH,
  APPROACH,
  HANDLE_OBJECT,
  LINE_SEARCH,
  LINE_FOLLOW
};

RobotState currentState = SEARCH;

void setup()
{
  Serial.begin(9600);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  initMotors();
  initUltrasonics();

  Serial.println("Bottle seeker system ready.");
}

void loop()
{
  // Sensor readings
  unsigned int dL = getLeftDistance();
  unsigned int dM = getMiddleDistance();
  unsigned int dR = getRightDistance();
  bool objAny = (dL < DIST_THRESHOLD) || (dM < DIST_THRESHOLD) || (dR < DIST_THRESHOLD);

  // Emergency stop if object is too close
  if (dM < GRIPPER_THRESHOLD)
  {
    stopAllMotors();
    Serial.println(F("Object too close. Activating gripper..."));
    delay(2000); // placeholder for gripper action
    return;
  }

  switch (currentState)
  {
  case SEARCH:
    if (!objAny)
    {
      driveForward();
      delay(100);
    }
    else
    {
      currentState = APPROACH;
    }
    break;

  case APPROACH:
    if (dL < DIST_THRESHOLD && dR >= DIST_THRESHOLD)
    {
      Serial.println("Rotate CCW");
      unsigned long start = millis();
      while (millis() - start < ROTATE_STEP_MS)
        pivotLeft();
    }
    else if (dR < DIST_THRESHOLD && dL >= DIST_THRESHOLD)
    {
      Serial.println("Rotate CW");
      unsigned long start = millis();
      while (millis() - start < ROTATE_STEP_MS)
        pivotRight();
    }

    // Re-check middle sensor after rotate
    dM = getMiddleDistance();
    if (dM < DIST_THRESHOLD)
    {
      Serial.println("Centered – driving forward");
      driveForward();
    }

    delay(100);
    break;

  case HANDLE_OBJECT:
  case LINE_SEARCH:
  case LINE_FOLLOW:
    stopAllMotors();
    // Not yet implemented
    break;

  default:
    stopAllMotors();
    break;
  }
}
