/*
  turret_stepper_demo_B.ino
  Serial‑interactive stepper test
  Type an integer angle (e.g. 45, -30) + <Enter>
*/

#include <Arduino.h>
#include <Stepper.h>

/* ── constants (same as Option A) ────────────────────────── */
const long MOTOR_STEPS_PER_REV = 200;
const int GEAR_NUM = 12;
const int GEAR_DEN = 35;
const float DEG_PER_STEP = 360.0f * GEAR_NUM / (MOTOR_STEPS_PER_REV * GEAR_DEN);

const uint8_t IN1 = 8, IN2 = 9, IN3 = 10, IN4 = 11;
const uint8_t TURRET_RPM = 20;

Stepper turretStepper(MOTOR_STEPS_PER_REV, IN1, IN3, IN2, IN4);

/* ── helper ──────────────────────────────────────────────── */
void turretRotate(float degrees)
{
  long steps = lroundf(degrees / DEG_PER_STEP);
  turretStepper.step(steps);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  } // wait on Leonardo/ProMicro
  turretStepper.setSpeed(TURRET_RPM);
  Serial.println(F("Stepper demo B – type angle then ENTER"));
  Serial.println(F("positive = CW, negative = CCW"));
}

void loop()
{
  if (Serial.available() > 0)
  {
    float angle = Serial.parseFloat(); // reads +/- float
    // flush the line
    while (Serial.available())
      Serial.read();

    Serial.print(F("→ rotating "));
    Serial.print(angle, 2);
    Serial.println(F(" degrees"));
    turretRotate(angle);
    Serial.println(F("✓ done"));
  }
  delay(100);
}