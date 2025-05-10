/*  ────────────────────────────────────────────────────────────
    grippers.cpp  –  4-arm bottle-handling demo
    • Stepper : L298N, 200 steps/rev, 20 RPM
    • EN_PIN driven from one PWM pin to cut idle current
    • Servos  : raiseElbow / lowerElbow  |  openClaw / closeClaw
    • Lift    : performed by raiseElbow(currentArm)
   ──────────────────────────────────────────────────────────── */

#include <Arduino.h>
#include <Servo.h>

/* ── stepper constants ─────────────────────────────────────── */
const int    STEPS_PER_REV  = 200;        // 1.8° motor
const uint8_t EN_PIN        = 6;         // PWM – enable both bridges

/* ── A4988 driver pins & gear ratio ───────────────────────── */
const int    DIR_PIN       = 2;           // STEP/DIR driver: DIR pin
const int    STEP_PIN      = 3;           // STEP pin
const float  gearRatio     = 225.0f / 90.0f;  // small gear : turret gear

/* ── servo pin map ─────────────────────────────────────────── */
const uint8_t ELBOW_PIN[4]   = {37, 41, 47, 49};
const uint8_t CLAW_PIN[4]    = {38, 42, 48, 50};
const uint8_t CAP_PUSHER_PIN = 39;        // microswitch test
// your switch pin
const uint8_t SWITCH_PIN = 7;

Servo elbowSrv[4], clawSrv[4], capSrv;

/* ── angle presets ─────────────────────────────────────────── */
const uint8_t ELBOW_UP_ANGLE    = 45;
const uint8_t ELBOW_DOWN_ANGLE  = 110;
const uint8_t CLAW_OPEN_ANGLE   = 30;
const uint8_t CLAW_CLOSE_ANGLE  = 180;
const uint8_t CAP_UP_ANGLE      = 60;
const uint8_t CAP_DOWN_ANGLE    = 150;

/* ── state machine ─────────────────────────────────────────── */
enum GState { PRE_GRAB, GRAB, TEST_CAP, STORE_LIFT, STORE_ROTATE, RESET_ARM };  
GState  state       = PRE_GRAB;
uint8_t currentArm  = 0;
uint8_t bottleCount = 0;

/* ── enable / disable helpers ─────────────────────────────── */
void enableStepper(uint8_t duty = 255)  // 255 = full torque
{
  analogWrite(EN_PIN, duty);
  delay(10);                          // wait for coils to energise
}

void disableStepper()                   // 0 = coils off
{
  analogWrite(EN_PIN, 0);
}

/* ── turret rotation using A4988 full-step ───────────────── */
void turretRotate(float desiredAngle) {
  enableStepper();
  // 1) compute equivalent small-gear angle
  float smallGearAngle = desiredAngle * gearRatio;

  // 2) convert to motor steps
  int stepsToMove = int((smallGearAngle / 360.0) * STEPS_PER_REV);

  // 3) step the motor
  digitalWrite(DIR_PIN, HIGH);            // HIGH = CW, LOW = CCW
  for (int i = 0; i < stepsToMove; ++i) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5000);
    digitalWrite(STEP_PIN, LOW);
    delay(5); // defines the rpm
  }
  delay(200); // wait for the motor to stop
  disableStepper();
}

/* ── claw helpers ──────────────────────────────────────────── */
void openClaw(uint8_t armIdx)
{
  clawSrv[armIdx].attach(CLAW_PIN[armIdx]);
  clawSrv[armIdx].write(CLAW_OPEN_ANGLE);
  delay(200);
  clawSrv[armIdx].detach();
}

void closeClaw(uint8_t armIdx)
{
  clawSrv[armIdx].attach(CLAW_PIN[armIdx]);
  clawSrv[armIdx].write(CLAW_CLOSE_ANGLE);
  delay(200);
  clawSrv[armIdx].detach();
}

/* ── elbow helpers ─────────────────────────────────────────── */
void raiseElbow(uint8_t armIdx)
{
  elbowSrv[armIdx].attach(ELBOW_PIN[armIdx]);
  elbowSrv[armIdx].write(ELBOW_UP_ANGLE);
  delay(200);
  elbowSrv[armIdx].detach();
}

void lowerElbow(uint8_t armIdx)
{
  elbowSrv[armIdx].attach(ELBOW_PIN[armIdx]);
  elbowSrv[armIdx].write(ELBOW_DOWN_ANGLE);
  delay(200);
  elbowSrv[armIdx].detach();
}

/* ── setup helpers ─────────────────────────────────────────── */
void setupServos()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    raiseElbow(i);    // start raised
    closeClaw(i);     // and closed
  }
  capSrv.attach(CAP_PUSHER_PIN);
  capSrv.write(CAP_UP_ANGLE);
  capSrv.detach();
}

void setupStepper()
{
  pinMode(EN_PIN,    OUTPUT);
  disableStepper();        // coils off at boot

  // A4988 control pins
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
}

/* ── Arduino lifecycle ────────────────────────────────────── */
void setup()
{
  Serial.begin(9600);
  setupServos();
  setupStepper();
  Serial.println(F("Gripper demo ready (idle current minimised)"));
}

void loop()
{
  switch (state)
  {
    case PRE_GRAB:
      Serial.println(F("PRE_GRAB → turret rotate 90° CCW"));
      openClaw(currentArm);
      lowerElbow(currentArm);
      delay(600);
      state = GRAB;
      break;

    case GRAB:
      Serial.println(F("GRAB → closing claw"));
      closeClaw(currentArm);
      delay(600);
      state = STORE_LIFT;
      break;

    case TEST_CAP:
      Serial.println(F("TEST_CAP → pressing & testing cap"));
      capSrv.attach(CAP_PUSHER_PIN);
      capSrv.write(CAP_DOWN_ANGLE);
      delay(700);

      // POLL the microswitch
      if (digitalRead(SWITCH_PIN) == LOW) {
        Serial.println(F("→ Cap detected"));
      } else {
        Serial.println(F("→ No cap detected"));
      }

      capSrv.write(CAP_UP_ANGLE);
      delay(250);
      capSrv.detach();

      state = STORE_LIFT;
      break;

    case STORE_LIFT:
      Serial.println(F("STORE_LIFT → raising elbow (individual lift)"));
      raiseElbow(currentArm);
      delay(400);
      state = STORE_ROTATE;
      break;

    case STORE_ROTATE:
      Serial.println(F("STORE_ROTATE → turret rotate 90° CW"));
      turretRotate(90.0f);
      state = RESET_ARM;
      break;

    case RESET_ARM:
      Serial.println(F("RESET_ARM → advance to next arm"));
      delay(400);
      bottleCount++;
      currentArm = (currentArm + 1) % 4;
      state = PRE_GRAB;
      break;
  }
}