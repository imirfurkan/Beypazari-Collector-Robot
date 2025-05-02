// #include "motor.h"
// #include "motionControl.h"
// #include <Arduino.h>

// // === Pin Definitions ===
// const int STBY = 2;

// const int PWM1 = 3, IN1_1 = 4, IN2_1 = 7;    // Motor 1 speed, direction pin 1, direction pin 2
// const int PWM2 = 5, IN1_2 = 8, IN2_2 = 12;   // Motor 2 speed, direction pin 1, direction pin 2
// const int PWM3 = 6, IN1_3 = 10, IN2_3 = 9;   // Motor 3 speed, direction pin 1, direction pin 2
// const int PWM4 = 11, IN1_4 = A0, IN2_4 = A1; // Motor 4 speed, direction pin 1 (A0 = digital 14),
//                                              // direction pin 2 (A1 = digital 15)

// // === Internal State ===
// static bool motorEnabled[4] = {true, true, true, true};
// static Direction motorDirection[4] = {FORWARD, FORWARD, FORWARD, FORWARD};
// static int baseSpeed = 100;
// static float K_unloaded[4] = {1.00, 0.83, 0.835, 0.75};
// static bool useTrim = true;

// static int trimmedPWM(int i)
// {
//   int pwmVal = baseSpeed;
//   if (useTrim)
//   {
//     pwmVal = int(constrain(baseSpeed * K_unloaded[i], 0, 255));
//   }
//   return pwmVal;
// }

// void initMotors()
// {
//   int pins[] = {STBY, PWM1,  IN1_1, IN2_1, PWM2,  IN1_2, IN2_2,
//                 PWM3, IN1_3, IN2_3, PWM4,  IN1_4, IN2_4};
//   for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++)
//   {
//     pinMode(pins[i], OUTPUT);
//   }

//   digitalWrite(STBY, HIGH);
// }

// void updateMotors()
// {
//   // Set direction pins based on runtime state
//   digitalWrite(IN1_1, motorDirection[0]);
//   digitalWrite(IN2_1, !motorDirection[0]);
//   digitalWrite(IN1_2, motorDirection[1]);
//   digitalWrite(IN2_2, !motorDirection[1]);
//   digitalWrite(IN1_3, motorDirection[2]);
//   digitalWrite(IN2_3, !motorDirection[2]);
//   digitalWrite(IN1_4, motorDirection[3]);
//   digitalWrite(IN2_4, !motorDirection[3]);

//   // Apply speed //	If enabled, send the trimmed speed; if disabled, send 0 (motor stops)
//   analogWrite(PWM1, motorEnabled[0] ? trimmedPWM(0) : 0);
//   analogWrite(PWM2, motorEnabled[1] ? trimmedPWM(1) : 0);
//   analogWrite(PWM3, motorEnabled[2] ? trimmedPWM(2) : 0);
//   analogWrite(PWM4, motorEnabled[3] ? trimmedPWM(3) : 0);
// }

// void setMotorEnabled(int i, bool enable)
// {
//   if (i >= 0 && i < 4)
//     motorEnabled[i] = enable;
// }

// void setMotorDirection(int i, Direction dir)
// {
//   if (i >= 0 && i < 4)
//     motorDirection[i] = dir;
// }

// void setKTrim(int i, float k)
// {
//   if (i >= 0 && i < 4 && k > 0.0)
//     K_unloaded[i] = k;
// }

// void setUseTrim(bool trimOn)
// {
//   useTrim = trimOn;
// }

// void setBaseSpeed(int speed)
// {
//   baseSpeed = constrain(speed, 0, 255);
// }

// void setStandby(bool enable)
// {
//   digitalWrite(STBY, enable ? HIGH : LOW);
// }

#include <Arduino.h>
#include <Arduino.h>
#include "motor.h"

const int STBY = 30;
const int PWM[4] = {2, 3, 4, 5};
const int IN1[4] = {22, 24, 26, 28};
const int IN2[4] = {23, 25, 27, 29};

int baseSpeed = 100;
float trimFactor[4] = {0.974, 0.808, 0.835, 0.750};

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
