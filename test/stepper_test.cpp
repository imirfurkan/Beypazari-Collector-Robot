#include <Arduino.h>
// pin connections
const int dirPin = 2;
const int stepPin = 3;
const int enablePin = 6; // A4988 EN pin (active LOW)

const int stepsPerRevolution = 200;
const float gearRatio = 225.0 / 90.0;
const float desiredAngle = 90.0;


void rotateAngle(float desiredAngle)
{
  // 1) compute equivalent small-gear angle
  float smallGearAngle = desiredAngle * gearRatio;

  // 2) convert to motor steps
  int stepsToMove = int((smallGearAngle / 360.0) * stepsPerRevolution);

  // 3) rotate motor
  digitalWrite(dirPin, HIGH); // HIGH = CW, LOW = CCW
  for (int i = 0; i < stepsToMove; i++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5000);
    digitalWrite(stepPin, LOW);
    // rpm
    delay(5);
  }
}

void setup()
{
  // in setup(), before anything else:
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW); // ENABLE driver (LOW = enabled)
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
}

void loop()
{
  rotateAngle(desiredAngle);
  delay(2000);
}


///////////// or try this

// #include <Arduino.h>

// // ────────────────────────────────────────────────────────────
// //  Stepper (A4988) test – turretRotate only
// // ────────────────────────────────────────────────────────────

// #pragma once

// // ── Hardware constants ───────────────────────────────────────
// static constexpr int EN_PIN         = 6;   // PWM – enable stepper coils
// static constexpr int DIR_PIN        = 2;   // A4988 DIR
// static constexpr int STEP_PIN       = 3;   // A4988 STEP
// static constexpr int STEPS_PER_REV  = 200; // 1.8° full steps per rev
// static constexpr float gearRatio    = 225.0f / 90.0f; // small-gear : turret-gear

// // ── Enable / disable helpers ─────────────────────────────────
// void enableStepper(uint8_t duty = 255) {
//   analogWrite(EN_PIN, duty);  // 255 = full torque
//   delay(10);                  // allow coils to energize
// }

// void disableStepper() {
//   analogWrite(EN_PIN, 0);     // 0 = coils off
// }

// // ── Turret rotation using A4988 full-step ─────────────────────
// void turretRotate(float desiredAngle) {
//   enableStepper();

//   // 1) compute equivalent small-gear angle
//   float smallGearAngle = desiredAngle * gearRatio;
//   // 2) convert to full steps
//   int stepsToMove = int((smallGearAngle / 360.0f) * STEPS_PER_REV);

//   Serial.print(F("turretRotate: "));
//   Serial.print(desiredAngle);
//   Serial.print(F("° → "));
//   Serial.print(stepsToMove);
//   Serial.println(F(" steps"));

//   // 3) drive the motor
//   digitalWrite(DIR_PIN, desiredAngle >= 0 ? HIGH : LOW); // CW if positive
//   for (int i = 0; i < stepsToMove; ++i) {
//     digitalWrite(STEP_PIN, HIGH);
//     delayMicroseconds(5000);
//     digitalWrite(STEP_PIN, LOW);
//     delay(5); // controls speed (≈20 RPM)
//   }

//   delay(200);      // settle time
//   disableStepper();
// }

// // ── Arduino lifecycle ───────────────────────────────────────
// void setup() {
//   Serial.begin(9600);
//   while (!Serial) { /* wait for Serial on some boards */ }

//   // init pins
//   pinMode(EN_PIN,   OUTPUT);
//   pinMode(DIR_PIN,  OUTPUT);
//   pinMode(STEP_PIN, OUTPUT);
//   digitalWrite(DIR_PIN, LOW);
//   disableStepper();

//   Serial.println(F("Stepper turret test ready"));
// }

// void loop() {
//   // rotate +90° CW
//   turretRotate(+90.0f);
//   delay(1000);

//   // rotate –90° CCW (back to start)
//   turretRotate(-90.0f);
//   delay(1000);
// }
