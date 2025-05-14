#include "grippers.h"
#include <Servo.h>

// ── Hardware constants ──────────────────────────────────────
static const int STEPS_PER_REV = 200; // 1.8° stepper
static const uint8_t EN_PIN = 11;     // PWM – enable coils
static const int DIR_PIN = 12;        // A4988 DIR
static const int STEP_PIN = 13;       // A4988 STEP
static const float gearRatio = 225.0f / 90.0f;
static const uint8_t NUM_ARMS = 2; // number of gripper arms

static const uint8_t ELBOW_PIN[2] = {36, 38};
static const uint8_t CLAW_PIN[2] = {37, 39};
static const uint8_t CAP_PUSHER_PIN = 40; // microswitch actuator
static const uint8_t SWITCH_PIN = 41;     // cap‐present switch

// ── Module‐local flag ───────────────────────────────────────
static bool lastRejected = false;

// ── Angle presets ─────────────────────────────────────────
static const uint8_t ELBOW_UP_ANGLE = 120;
static const uint8_t ELBOW_DOWN_ANGLE = 180;
static const uint8_t CLAW_OPEN_ANGLE = 30;
static const uint8_t CLAW_CLOSE_ANGLE = 180;
static const uint8_t CAP_UP_ANGLE = 85;        // TODO
static const uint8_t CAP_MIDDLE_ANGLE = 165;   // TODO
static const uint8_t CAP_DOWN_ANGLE = 175;     // TODO
static const uint8_t BOTTLE_CHECK_ANGLE = 180; // TODO

static const uint8_t desiredAngle = 93.0f;
// ── Module state ───────────────────────────────────────────
enum GripperState
{
  GRAB,
  TEST_CAP,
  STORE,
  RESET_ARM,
  REJECT_BOTTLE
};
static GripperState gripperState = GRAB;
static uint8_t currentArm = 0;
uint8_t bottleCount = 0;

// ── Servo objects ──────────────────────────────────────────
static Servo elbowSrv[2], clawSrv[2], capSrv;

// ── Private helpers ────────────────────────────────────────
static void enableStepper()
{
  digitalWrite(EN_PIN, LOW); // LOW = enabled (active-LOW)
  delay(10);
}

static void disableStepper()
{
  digitalWrite(EN_PIN, HIGH); // HIGH = disabled
}

static void turretRotate(float angle)
{
  enableStepper();
  // The current logic is set up correctly for arm 0, so invert direction for arm 1.
  if (currentArm == 1)
  {
    angle = -angle;
  }
  // CW for positive angles, CCW for negative
  bool directionCW = (angle >= 0);
  digitalWrite(DIR_PIN, directionCW ? HIGH : LOW);
  // convert angle to steps
  int steps = abs(int((angle * gearRatio / 360.0f) * STEPS_PER_REV));
  // digitalWrite(DIR_PIN, HIGH);
  for (int i = 0; i < steps; ++i)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5000);
    digitalWrite(STEP_PIN, LOW);
    delay(5); // sets the pwm
  }
  delay(200);
  // we disable the stepper driver here to avoid constant current
  // draw when the arm is not moving and overheating the driver
  // disableStepper(); // TODO disable et? akım?
}
static void openClaw(uint8_t i)
{
  clawSrv[i].attach(CLAW_PIN[i]);
  clawSrv[i].write(CLAW_OPEN_ANGLE);
  delay(700);
  // clawSrv[i].detach();
}
static void closeClaw(uint8_t i)
{
  clawSrv[i].attach(CLAW_PIN[i]);
  clawSrv[i].write(CLAW_CLOSE_ANGLE);
  delay(700);
  // clawSrv[i].detach();
}
static void raiseElbow(uint8_t i)
{
  elbowSrv[i].attach(ELBOW_PIN[i]);
  elbowSrv[i].write(ELBOW_UP_ANGLE);
  delay(700);
  // elbowSrv[i].detach();
}
static void lowerElbow(uint8_t i)
{
  elbowSrv[i].attach(ELBOW_PIN[i]);
  elbowSrv[i].write(ELBOW_DOWN_ANGLE);
  delay(700);
  // elbowSrv[i].detach();
}
static void setupServos()
{
  for (uint8_t i = 0; i < NUM_ARMS; ++i)
  {
    raiseElbow(i);
    closeClaw(i);
  }
  capSrv.attach(CAP_PUSHER_PIN);
  capSrv.write(CAP_UP_ANGLE);
  // capSrv.detach();
}
static void setupStepper()
{
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // ENABLE driver (LOW = enabled)
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  // disableStepper(); //TODO enable hep?
}

// ── Public API ──────────────────────────────────────────────
// ─ Public API ──────────────────────────────────────────────
bool bottleRejected()
{
  return lastRejected;
}

void Grippers_openClaws()
{
  for (uint8_t i = 0; i < NUM_ARMS; ++i)
  {
    openClaw(i);
  }
}

void Grippers_closeClaws()
{
  for (uint8_t i = 0; i < NUM_ARMS; ++i)
  {
    closeClaw(i);
  }
}

void Grippers_lowerElbows()
{
  for (uint8_t i = 0; i < NUM_ARMS; ++i)
  {
    lowerElbow(i);
  }
}

void Grippers_raiseElbows()
{
  for (uint8_t i = 0; i < NUM_ARMS; ++i)
  {
    raiseElbow(i);
  }
}

void Grippers_setup()
{
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  setupServos();
  setupStepper();
}

bool Grippers_loop()
{
  switch (gripperState)
  {
  case GRAB:
    turretRotate(desiredAngle); // rotate to grab
    delay(1000);                // TODO lazim mi
    openClaw(currentArm);
    delay(600);
    lowerElbow(currentArm);
    delay(600);

    closeClaw(currentArm);
    delay(600);
    gripperState = TEST_CAP;
    return false;

  case TEST_CAP:
  {
    capSrv.attach(CAP_PUSHER_PIN);
    capSrv.write(CAP_MIDDLE_ANGLE);
    delay(3000);

    // step down from CAP_UP_ANGLE → CAP_DOWN_ANGLE
    // increase the delay here to make it slower
    for (uint8_t ang = CAP_MIDDLE_ANGLE; ang <= CAP_DOWN_ANGLE; ang++)
    {
      capSrv.write(ang);
      delay(60); // ← 20ms per degree ~ slower; make larger to go even slower
    }
    delay(1000);
    bool capPresent = (digitalRead(SWITCH_PIN) == LOW);

    bool bottlePresent = false;
    if (!capPresent)
    {
      for (uint8_t ang = CAP_DOWN_ANGLE; ang <= BOTTLE_CHECK_ANGLE; ang++)
      {
        capSrv.write(ang);
        delay(60); // ← 20ms per degree ~ slower; make larger to go even slower
      }
      delay(1000);
      bottlePresent = (digitalRead(SWITCH_PIN) == LOW);
    }

    // step back up from CAP_DOWN_ANGLE → CAP_UP_ANGLE
    for (int ang = CAP_DOWN_ANGLE; ang >= CAP_UP_ANGLE; ang--)
    {
      capSrv.write(ang);
      delay(20); // same stepping delay
    }

    delay(2000);

    if (bottlePresent)
    {
      gripperState = STORE;
    }
    else
    {
      gripperState = REJECT_BOTTLE;
    }
    return false;
  }

  case STORE:
    raiseElbow(currentArm);
    delay(400);
    turretRotate(-desiredAngle);
    delay(1000); // TODO lazim mi
    gripperState = RESET_ARM;
    return false;

  case RESET_ARM:
    delay(400);
    bottleCount++;
    currentArm = (currentArm + 1) % NUM_ARMS;
    lastRejected = false;
    gripperState = GRAB;
    return true; // one full cycle done

  case REJECT_BOTTLE:
    // reject flow: open, lift, close, advance arm, no count
    openClaw(currentArm);
    delay(1500);
    raiseElbow(currentArm);
    delay(1500);
    closeClaw(currentArm);
    delay(1500);
    turretRotate(-desiredAngle);
    delay(1000); // TODO lazim mi
    currentArm = (currentArm + 1) % NUM_ARMS;
    lastRejected = true;
    gripperState = GRAB;
    return true;
  }
  return false;
}

// // ── Test code for main.cpp ───────────────────────────────────────────────
// #include <Arduino.h>
// #include "Grippers.h"

// void setup()
// {
//   // start serial for debug (optional)
//   Serial.begin(115200);
//   while (!Serial)
//   { /* wait for Serial on some boards */
//   }

//   // initialize gripper hardware and state
//   Grippers_setup();
// }

// void loop()
// {
//   // call the gripper state-machine; if it's still mid-cycle, bail out immediately
//   if (!Grippers_loop())
//   {
//     return;
//   }

//   // ← here only when Grippers_loop() has returned true
//   Serial.println(F("Gripper cycle complete!"));
// }
