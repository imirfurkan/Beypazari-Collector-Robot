
#include "motors.h"

// ── Internal state ────────────────────────────────────────
static int s_baseSpeed = MOTOR_SPEED_DEFAULT;

// Compute the PWM for motor `idx` after applying its unloaded factor
static int trimmedPWM(uint8_t idx)
{
  return constrain(int(s_baseSpeed * MOTOR_K_UNLOADED[idx]), 0, 255);
}

// Low-level: drive a single motor idx either forward or reverse
static void driveMotor(uint8_t idx, bool forward)
{
  digitalWrite(MOTOR_IN1_PIN[idx], forward ? HIGH : LOW);
  digitalWrite(MOTOR_IN2_PIN[idx], forward ? LOW : HIGH);
  analogWrite(MOTOR_PWM_PIN[idx], trimmedPWM(idx));
}

// ── Public functions ──────────────────────────────────────
void Motor_setup()
{
  // Enable TB6612
  pinMode(MOTOR_STBY_PIN, OUTPUT);
  digitalWrite(MOTOR_STBY_PIN, HIGH);

  // Configure each motor’s pins
  for (uint8_t i = 0; i < 4; ++i)
  {
    pinMode(MOTOR_PWM_PIN[i], OUTPUT);
    pinMode(MOTOR_IN1_PIN[i], OUTPUT);
    pinMode(MOTOR_IN2_PIN[i], OUTPUT);
  }
}

void Motor_setBaseSpeed(int speed) // TODO
{
  s_baseSpeed = constrain(speed, 0, 255);
}

void Motor_stopAll() // TODO STBY kapamalı yap
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    analogWrite(MOTOR_PWM_PIN[i], 0);
  }
}

void Motor_driveForward()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    driveMotor(i, true);
  }
}

void Motor_driveBackward()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    driveMotor(i, false);
  }
}

void Motor_rotateCW()
{
  // motors 0&1 spin “forward”, 2&3 spin “reverse”
  for (uint8_t i = 0; i < 4; ++i)
  {
    bool dir = (i < 2) ? MOTOR_FORWARD_DIR : !MOTOR_FORWARD_DIR;
    driveMotor(i, dir);
  }
}

void Motor_rotateCCW()
{
  // inverse of rotateCW()
  for (uint8_t i = 0; i < 4; ++i)
  {
    bool dir = (i < 2) ? !MOTOR_FORWARD_DIR : MOTOR_FORWARD_DIR;
    driveMotor(i, dir);
  }
}

void Motor_gentleLeft()
{
  int leftPWM = int(MOTOR_GENTLE_SPEED * (1.0f - MOTOR_STEER_PCT));
  int rightPWM = MOTOR_GENTLE_SPEED;

  for (uint8_t i = 0; i < 4; ++i)
  {
    digitalWrite(MOTOR_IN1_PIN[i], HIGH);
    digitalWrite(MOTOR_IN2_PIN[i], LOW);
    analogWrite(MOTOR_PWM_PIN[i], (i < 2) ? leftPWM : rightPWM);
  }
}

void Motor_gentleRight()
{
  int leftPWM = MOTOR_GENTLE_SPEED;
  int rightPWM = int(MOTOR_GENTLE_SPEED * (1.0f - MOTOR_STEER_PCT));

  for (uint8_t i = 0; i < 4; ++i)
  {
    digitalWrite(MOTOR_IN1_PIN[i], HIGH);
    digitalWrite(MOTOR_IN2_PIN[i], LOW);
    analogWrite(MOTOR_PWM_PIN[i], (i < 2) ? leftPWM : rightPWM);
  }
}
