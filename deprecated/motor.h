// #pragma once

// enum Direction
// {
//   FORWARD = 1,
//   REVERSE = 0
// };

// void initMotors();
// void updateMotors();

// void setMotorEnabled(int i, bool enable);
// void setMotorDirection(int i, Direction dir);
// void setKTrim(int i, float k);
// void setUseTrim(bool trimOn);
// void setBaseSpeed(int speed);
// void setStandby(bool enable);

#pragma once

enum Direction
{
  FORWARD = 1,
  REVERSE = 0
};

void initMotors();
void driveMotor(int idx, Direction dir);
void setMotorPWM(int idx, int pwmVal);
void stopAllMotors();

extern int baseSpeed;
extern float trimFactor[4];
