#pragma once

enum MotionMode
{
  MOVE_FORWARD,
  MOVE_BACKWARD,
  MOVE_LEFT,
  MOVE_RIGHT,
  ROTATE_LEFT,
  ROTATE_RIGHT,
  RETURN_MODE,
  STOP
};

extern MotionMode currentMode;

void setMotionMode(MotionMode mode);