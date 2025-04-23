#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#include <avr/io.h>
#include <util/delay.h>
// === Pin Definitions ===
// Shared standby (active HIGH)
const int STBY = 2;

// Driver A → Motor 1 & Motor 2
const int PWM1 = 3;  // Motor 1 speed (PWM)
const int IN1_1 = 4; // Motor 1 direction pin 1
const int IN2_1 = 7; // Motor 1 direction pin 2

const int PWM2 = 5;   // Motor 2 speed (PWM)
const int IN1_2 = 8;  // Motor 2 direction pin 1
const int IN2_2 = 12; // Motor 2 direction pin 2

// Driver B → Motor 3 & Motor 4
const int PWM3 = 6;   // Motor 3 speed (PWM)
const int IN1_3 = 10; // Motor 3 direction pin 1
const int IN2_3 = 11; // Motor 3 direction pin 2

const int PWM4 = 9;   // Motor 4 speed (PWM)
const int IN1_4 = A0; // Motor 4 direction pin 1 (A0 = digital 14)
const int IN2_4 = A1; // Motor 4 direction pin 2 (A1 = digital 15)

// === CONFIG: which motors you want ACTIVE? ===
bool motorEnabled[4] = {
    true, // Motor 1
    true, // Motor 2
    true, // Motor 3
    true  // Motor 4
};

// === CONFIG: base speed and trim gains ===
const int baseSpeed = 100; // 0–255

// Your calibrated scale factors (K) per motor:
//   K[i] = (max observed speed) / (measured speed_i)
// fill these with your measured values:
float K_unloaded[4] = {1.00, 0.83, 0.835, 0.75};

// Flip to use trim compensation or not:
bool useTrim = true;

// Choose direction once for all:
const bool forwardDir = true; // true=IN1=HIGH/IN2=LOW, false=reversed

// === HELPER: compute the PWM for motor i ===
int trimmedPWM(int i)
{
  int pwmVal = baseSpeed;
  if (useTrim)
  {
    pwmVal = int(constrain(baseSpeed * K_unloaded[i], 0, 255));
  }
  return pwmVal;
}

void setup()
{
  // Configure all control pins as outputs
  int pins[] = {STBY, PWM1,  IN1_1, IN2_1, PWM2,  IN1_2, IN2_2,
                PWM3, IN1_3, IN2_3, PWM4,  IN1_4, IN2_4};
  for (int idx = 0; idx < sizeof(pins) / sizeof(pins[0]); idx++)
  {
    pinMode(pins[idx], OUTPUT);
  }

  // Exit standby
  digitalWrite(STBY, HIGH);

  // Pre-set all directions
  digitalWrite(IN1_1, forwardDir);
  digitalWrite(IN2_1, !forwardDir);
  digitalWrite(IN1_2, forwardDir);
  digitalWrite(IN2_2, !forwardDir);
  digitalWrite(IN1_3, forwardDir);
  digitalWrite(IN2_3, !forwardDir);
  digitalWrite(IN1_4, forwardDir);
  digitalWrite(IN2_4, !forwardDir);
}

void loop()
{
  // Motor 1
  analogWrite(PWM1, motorEnabled[0] ? trimmedPWM(0) : 0);
  // Motor 2
  analogWrite(PWM2, motorEnabled[1] ? trimmedPWM(1) : 0);
  // Motor 3
  analogWrite(PWM3, motorEnabled[2] ? trimmedPWM(2) : 0);
  // Motor 4
  analogWrite(PWM4, motorEnabled[3] ? trimmedPWM(3) : 0);

  // nothing else to do—motors run continuously.
  // To change which motors run, or K values, edit above & re-upload.
}