#include "motionControl.h"
#include "motor.h" // so we can call setMotorEnabled, setMotorDirection

MotionMode currentMode = STOP;

void setMotionMode(MotionMode mode)
{
  currentMode = mode;

  switch (mode)
  {
  case MOVE_BACKWARD:
    setStandby(true);
    for (int i = 0; i < 4; i++)
    {
      setMotorEnabled(i, true);
      setMotorDirection(i, FORWARD); // <-- this is Direction::FORWARD
    }
    break;

  case MOVE_FORWARD:
    setStandby(true);
    for (int i = 0; i < 4; i++)
    {
      setMotorEnabled(i, true);
      setMotorDirection(i, REVERSE);
    }
    break;

  case MOVE_RIGHT:
    setStandby(true);
    setMotorEnabled(0, true);
    setMotorDirection(0, FORWARD);
    setMotorEnabled(1, true);
    setMotorDirection(1, REVERSE);
    setMotorEnabled(2, true);
    setMotorDirection(2, FORWARD);
    setMotorEnabled(3, true);
    setMotorDirection(3, REVERSE);
    break;

  case MOVE_LEFT:
    setStandby(true);
    setMotorEnabled(0, true);
    setMotorDirection(0, REVERSE);
    setMotorEnabled(1, true);
    setMotorDirection(1, FORWARD);
    setMotorEnabled(2, true);
    setMotorDirection(2, REVERSE);
    setMotorEnabled(3, true);
    setMotorDirection(3, FORWARD);
    break;

  case ROTATE_RIGHT:
    setStandby(true);
    setMotorEnabled(0, true);
    setMotorDirection(0, FORWARD);
    setMotorEnabled(1, true);
    setMotorDirection(1, FORWARD);
    setMotorEnabled(2, true);
    setMotorDirection(2, REVERSE);
    setMotorEnabled(3, true);
    setMotorDirection(3, REVERSE);
    break;

  case ROTATE_LEFT:
    setStandby(true);
    setMotorEnabled(0, true);
    setMotorDirection(0, REVERSE);
    setMotorEnabled(1, true);
    setMotorDirection(1, REVERSE);
    setMotorEnabled(2, true);
    setMotorDirection(2, FORWARD);
    setMotorEnabled(3, true);
    setMotorDirection(3, FORWARD);
    break;

  case STOP:
  default:
    for (int i = 0; i < 4; i++)
    {
      setMotorEnabled(i, false);
    }
    setStandby(false);
    break;
  }
}