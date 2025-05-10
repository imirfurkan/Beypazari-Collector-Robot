#include <Arduino.h>
#include <Arduino.h>
#include "motor.h"

const int STBY = 30;
const int PWM[4] = {2, 3, 4, 5};
const int IN1[4] = {22, 24, 26, 28};
const int IN2[4] = {23, 25, 27, 29};

int baseSpeed = 100;
float trimFactor[4] = {0.974, 0.808, 0.835, 0.750}; // P controller

int trimmedPWM(int idx)
{
  return int(constrain(baseSpeed * trimFactor[idx], 0, 255));
}

void initMotors()
{
  pinMode(STBY, OUTPUT);
  for (int i = 0; i < 4; i++)
  {
    pinMode(PWM[i], OUTPUT);
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
  }
  digitalWrite(STBY, HIGH);
}

void driveMotor(int idx, Direction dir)
{
  digitalWrite(IN1[idx], dir == FORWARD ? HIGH : LOW);
  digitalWrite(IN2[idx], dir == FORWARD ? LOW : HIGH);
  analogWrite(PWM[idx], trimmedPWM(idx));
}

void setMotorPWM(int idx, int pwmVal)
{
  analogWrite(PWM[idx], pwmVal);
}

void stopAllMotors()
{
  for (int i = 0; i < 4; i++)
  {
    analogWrite(PWM[i], 0);
  }
}
