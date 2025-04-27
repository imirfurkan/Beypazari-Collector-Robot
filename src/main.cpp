#include <Arduino.h>
#include "motor.h"
#include "motionControl.h"

void setup()
{
  initMotors();
}

void loop()
{
  setMotionMode(MOVE_FORWARD);
  updateMotors();
  delay(5000);

  setMotionMode(MOVE_BACKWARD);
  updateMotors();
  delay(5000);

  setMotionMode(MOVE_LEFT);
  updateMotors();
  delay(5000);

  setMotionMode(MOVE_RIGHT);
  updateMotors();
  delay(5000);

  setMotionMode(ROTATE_LEFT);
  updateMotors();
  delay(5000);

  setMotionMode(ROTATE_RIGHT);
  updateMotors();
  delay(5000);

  setMotionMode(STOP);
  updateMotors();
  delay(5000);
}