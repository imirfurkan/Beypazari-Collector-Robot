// #include "motionControl.h"
// #include "motor.h" // so we can call setMotorEnabled, setMotorDirection

// MotionMode currentMode = STOP;

// void setMotionMode(MotionMode mode)
// {
//   currentMode = mode;

//   switch (mode)
//   {
//   case MOVE_BACKWARD:
//     setStandby(true);
//     for (int i = 0; i < 4; i++)
//     {
//       setMotorEnabled(i, true);
//       setMotorDirection(i, FORWARD); // <-- this is Direction::FORWARD
//     }
//     break;

//   case MOVE_FORWARD:
//     setStandby(true);
//     for (int i = 0; i < 4; i++)
//     {
//       setMotorEnabled(i, true);
//       setMotorDirection(i, REVERSE);
//     }
//     break;

//   case MOVE_RIGHT:
//     setStandby(true);
//     setMotorEnabled(0, true);
//     setMotorDirection(0, FORWARD);
//     setMotorEnabled(1, true);
//     setMotorDirection(1, REVERSE);
//     setMotorEnabled(2, true);
//     setMotorDirection(2, FORWARD);
//     setMotorEnabled(3, true);
//     setMotorDirection(3, REVERSE);
//     break;

//   case MOVE_LEFT:
//     setStandby(true);
//     setMotorEnabled(0, true);
//     setMotorDirection(0, REVERSE);
//     setMotorEnabled(1, true);
//     setMotorDirection(1, FORWARD);
//     setMotorEnabled(2, true);
//     setMotorDirection(2, REVERSE);
//     setMotorEnabled(3, true);
//     setMotorDirection(3, FORWARD);
//     break;

//   case ROTATE_RIGHT:
//     setStandby(true);
//     setMotorEnabled(0, true);
//     setMotorDirection(0, FORWARD);
//     setMotorEnabled(1, true);
//     setMotorDirection(1, FORWARD);
//     setMotorEnabled(2, true);
//     setMotorDirection(2, REVERSE);
//     setMotorEnabled(3, true);
//     setMotorDirection(3, REVERSE);
//     break;

//   case ROTATE_LEFT:
//     setStandby(true);
//     setMotorEnabled(0, true);
//     setMotorDirection(0, REVERSE);
//     setMotorEnabled(1, true);
//     setMotorDirection(1, REVERSE);
//     setMotorEnabled(2, true);
//     setMotorDirection(2, FORWARD);
//     setMotorEnabled(3, true);
//     setMotorDirection(3, FORWARD);
//     break;

//   case STOP:
//   default:
//     for (int i = 0; i < 4; i++)
//     {
//       setMotorEnabled(i, false);
//     }
//     setStandby(false);
//     break;
//   }
// }
#include <Arduino.h>
#include "motor.h"
#include "motionControl.h"

const float steerPct = 0.15;
const int pivotMs = 400;

void driveForward()
{
  for (int i = 0; i < 4; i++)
  {
    driveMotor(i, FORWARD);
  }
}

void gentleLeft()
{
  int leftPWM = int(baseSpeed * (1.0 - steerPct));
  int rightPWM = baseSpeed;

  // Left motors (0 and 1) go forward at reduced speed
  driveMotor(0, FORWARD);
  setMotorPWM(0, leftPWM);
  driveMotor(1, FORWARD);
  setMotorPWM(1, leftPWM);

  // Right motors (2 and 3) go forward at full speed
  driveMotor(2, FORWARD);
  setMotorPWM(2, rightPWM);
  driveMotor(3, FORWARD);
  setMotorPWM(3, rightPWM);
}

void gentleRight()
{
  int leftPWM = baseSpeed;
  int rightPWM = int(baseSpeed * (1.0 - steerPct));

  // Left motors (0 and 1) full speed
  driveMotor(0, FORWARD);
  setMotorPWM(0, leftPWM);
  driveMotor(1, FORWARD);
  setMotorPWM(1, leftPWM);

  // Right motors (2 and 3) reduced speed
  driveMotor(2, FORWARD);
  setMotorPWM(2, rightPWM);
  driveMotor(3, FORWARD);
  setMotorPWM(3, rightPWM);
}

void pivotLeft()
{
  driveMotor(0, REVERSE);
  driveMotor(1, REVERSE);
  driveMotor(2, FORWARD);
  driveMotor(3, FORWARD);
  delay(pivotMs);
}

void pivotRight()
{
  driveMotor(0, FORWARD);
  driveMotor(1, FORWARD);
  driveMotor(2, REVERSE);
  driveMotor(3, REVERSE);
  delay(pivotMs);
}

void applyMotion(MotionMode mode)
{
  switch (mode)
  {
  case DRIVE_FORWARD:
    driveForward();
    break;
  case GENTLE_LEFT:
    gentleLeft();
    break;
  case GENTLE_RIGHT:
    gentleRight();
    break;
  case PIVOT_LEFT:
    stopAllMotors();
    pivotLeft();
    driveForward();
    break;
  case PIVOT_RIGHT:
    stopAllMotors();
    pivotRight();
    driveForward();
    break;
  case STOP:
  default:
    stopAllMotors();
    break;
  }
}
