
#include "motors.h"

// ── Internal state ────────────────────────────────────────
static int s_baseSpeed = MOTOR_SPEED_DEFAULT;
static bool stbyEnabled = false; // Track standby pin enable state

// Ensure the TB6612 driver is enabled before any motor operation
static inline void ensureEnabled()
{
  if (!stbyEnabled)
  {
    digitalWrite(MOTOR_STBY_PIN, HIGH);
    delayMicroseconds(10); // Small delay to ensure driver is ready
    stbyEnabled = true;
  }
}

// Compute the PWM for motor `idx` after applying its unloaded factor
static int trimmedPWM(uint8_t idx)
{
  return constrain(int(s_baseSpeed * MOTOR_K_UNLOADED[idx]), 0, 255);
}

// Low-level: drive a single motor idx either forward or reverse
static void driveMotor(uint8_t idx, bool forward)
{
  ensureEnabled(); // Make sure driver is enabled before sending commands

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

void Motor_stopAll() // also disables STBY
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    analogWrite(MOTOR_PWM_PIN[i], 0);
  }

  digitalWrite(MOTOR_STBY_PIN, LOW);
  stbyEnabled = false;
}

void Motor_driveForward()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    driveMotor(i, false);
  }
}

void Motor_driveBackward()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    driveMotor(i, true);
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

void Motor_Left()
{
  int leftPWM = int(MOTOR_GENTLE_SPEED * (1.0f - MOTOR_STEER_PCT_SLIDE));
  int rightPWM = MOTOR_GENTLE_SPEED;

  for (uint8_t i = 0; i < 4; ++i)
  {
    digitalWrite(MOTOR_IN1_PIN[i], HIGH);
    digitalWrite(MOTOR_IN2_PIN[i], LOW);
    analogWrite(MOTOR_PWM_PIN[i], (i < 2) ? leftPWM : rightPWM);
  }
}

void Motor_Right()
{
  int leftPWM = MOTOR_GENTLE_SPEED;
  int rightPWM = int(MOTOR_GENTLE_SPEED * (1.0f - MOTOR_STEER_PCT_SLIDE));

  for (uint8_t i = 0; i < 4; ++i)
  {
    digitalWrite(MOTOR_IN1_PIN[i], HIGH);
    digitalWrite(MOTOR_IN2_PIN[i], LOW);
    analogWrite(MOTOR_PWM_PIN[i], (i < 2) ? leftPWM : rightPWM);
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

// // TEST CODE FOR MAIN.CPP

// #include "motors.h"

// // ── Setup: initialize pins and driver ──────────────────────
// void setup() {
//   Motor_setup();
// }

// // ── Main loop: forward → CW → CCW, each for 5 s ───────────
// void loop() {
//   // 1) Go straight ahead
//   Motor_driveForward();
//   delay(5000);  // 5000 ms = 5 s

//   // 2) Spin in place clockwise
//   Motor_rotateCW();
//   delay(5000);

//   // 3) Spin in place counter-clockwise
//   Motor_rotateCCW();
//   delay(5000);

//   // then immediately repeats…
// }