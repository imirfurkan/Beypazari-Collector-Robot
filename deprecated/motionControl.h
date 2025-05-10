#pragma once

enum MotionMode
{
  DRIVE_FORWARD,
  GENTLE_LEFT,
  GENTLE_RIGHT,
  PIVOT_LEFT,
  PIVOT_RIGHT,
  STOP
};

void driveForward();
void gentleLeft();
void gentleRight();
void pivotLeft();
void pivotRight();
void applyMotion(MotionMode mode);
