#pragma once

enum Direction
{
  FORWARD = 1,
  REVERSE = 0
};

void initMotors();
void updateMotors();

void setMotorEnabled(int i, bool enable);
void setMotorDirection(int i, Direction dir);
void setKTrim(int i, float k);
void setUseTrim(bool trimOn);
void setBaseSpeed(int speed);