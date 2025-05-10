#include <Arduino.h>

// ────────────────────────────────────────────────
//  Pin map (Mega) – unchanged *physical* wiring
// ────────────────────────────────────────────────
const int STBY = 30; // TB6612 standby

// Motors
const int PWM1 = 2;
const int IN1_1 = 22;
const int IN2_1 = 23; // back-left
const int PWM2 = 3;
const int IN1_2 = 24;
const int IN2_2 = 25; // front-left
const int PWM3 = 4;
const int IN1_3 = 26;
const int IN2_3 = 27; // front-right
const int PWM4 = 5;
const int IN1_4 = 28;
const int IN2_4 = 29; // back-right

// Line follower 5-channel (analog)
const int L1 = A0, L2 = A1, L3 = A2, L4 = A3, L5 = A4;

// HC-SR04 triggers/echos
const int TRIG_LEFT = 31;
const int ECHO_LEFT = 32;
const int TRIG_MIDDLE = 33;
const int ECHO_MIDDLE = 34;
const int TRIG_RIGHT = 35;
const int ECHO_RIGHT = 36;

// ────────────────────────────────────────────────
//  Parameters
// ────────────────────────────────────────────────
const unsigned int DIST_THRESHOLD = 80; // cm, for obstacle‐avoid in LINE_SEARCH
const int baseSpeed = 100;              // 0–255
float K_unloaded[4] = {0.974, 0.808, 0.835, 0.750};
const bool forwardDir = true;

const float steerPct = 0.15;        // ±15% gentle steer
const float threshold = 0.30;       // normalized IR threshold
const unsigned long BLAST_MS = 500; // “blast” forward after first line hit
const unsigned long PIVOT_MS = 400; // open‐loop pivot

// ────────────────────────────────────────────────
//  State machines
// ────────────────────────────────────────────────
enum Mode
{
  LINE_SEARCH,
  LINE_FOLLOW
} mode = LINE_SEARCH;
enum LSState
{
  LS_ROLL,
  LS_BLAST,
  LS_PIVOT,
  LS_DONE
} lsState = LS_ROLL;

// for LS timing and side flag
unsigned long lsStamp = 0;
int lsCounter = 0; // –1 = left‐first, +1 = right‐first

// obstacle avoid non-blocking
unsigned long lastObsChk = 0;
const unsigned long OBS_DT = 50;
bool oaActive = false;

// ────────────────────────────────────────────────
//  Helpers
// ────────────────────────────────────────────────
int trimmedPWM(int i)
{
  return constrain(int(baseSpeed * K_unloaded[i]), 0, 255);
}

void driveMotor(int idx, int pwm, int in1, int in2, bool fw)
{
  digitalWrite(in1, fw ? HIGH : LOW);
  digitalWrite(in2, fw ? LOW : HIGH);
  analogWrite(pwm, trimmedPWM(idx));
}
void stopAll()
{
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  analogWrite(PWM3, 0);
  analogWrite(PWM4, 0);
}
void driveForward()
{
  driveMotor(0, PWM1, IN1_1, IN2_1, true);
  driveMotor(1, PWM2, IN1_2, IN2_2, true);
  driveMotor(2, PWM3, IN1_3, IN2_3, true);
  driveMotor(3, PWM4, IN1_4, IN2_4, true);
}
void rotateCW()
{
  driveMotor(0, PWM1, IN1_1, IN2_1, !forwardDir);
  driveMotor(1, PWM2, IN1_2, IN2_2, !forwardDir);
  driveMotor(2, PWM3, IN1_3, IN2_3, forwardDir);
  driveMotor(3, PWM4, IN1_4, IN2_4, forwardDir);
}
void rotateCCW()
{
  driveMotor(0, PWM1, IN1_1, IN2_1, forwardDir);
  driveMotor(1, PWM2, IN1_2, IN2_2, forwardDir);
  driveMotor(2, PWM3, IN1_3, IN2_3, !forwardDir);
  driveMotor(3, PWM4, IN1_4, IN2_4, !forwardDir);
}

void gentleLeft()
{
  int leftPWM = int(baseSpeed * (1.0 - steerPct));
  int rightPWM = baseSpeed;
  // left side wheels slower
  digitalWrite(IN1_1, HIGH);
  digitalWrite(IN2_1, LOW);
  analogWrite(PWM1, leftPWM);
  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  analogWrite(PWM2, leftPWM);
  // right side full speed
  digitalWrite(IN1_3, HIGH);
  digitalWrite(IN2_3, LOW);
  analogWrite(PWM3, rightPWM);
  digitalWrite(IN1_4, HIGH);
  digitalWrite(IN2_4, LOW);
  analogWrite(PWM4, rightPWM);
}

void gentleRight()
{
  int leftPWM = baseSpeed;
  int rightPWM = int(baseSpeed * (1.0 - steerPct));
  digitalWrite(IN1_1, HIGH);
  digitalWrite(IN2_1, LOW);
  analogWrite(PWM1, leftPWM);
  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  analogWrite(PWM2, leftPWM);
  digitalWrite(IN1_3, HIGH);
  digitalWrite(IN2_3, LOW);
  analogWrite(PWM3, rightPWM);
  digitalWrite(IN1_4, HIGH);
  digitalWrite(IN2_4, LOW);
  analogWrite(PWM4, rightPWM);
}

unsigned int getDistance(int trig, int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 30000UL);
  return us ? us / 58 : 999; // cm
}

// ────────────────────────────────────────────────
//  Obstacle-avoid (only in LINE_SEARCH, always pivot CW)
// ────────────────────────────────────────────────
void updateObstacleAvoid()
{
  if (mode != LINE_SEARCH)
    return;
  unsigned long now = millis();
  if (now - lastObsChk < OBS_DT)
    return;
  lastObsChk = now;

  bool tooClose = (getDistance(TRIG_LEFT, TRIG_LEFT) >= 0, false);
  unsigned int dL = getDistance(TRIG_LEFT, ECHO_LEFT);
  unsigned int dM = getDistance(TRIG_MIDDLE, ECHO_MIDDLE);
  unsigned int dR = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  tooClose = (dL < DIST_THRESHOLD) || (dM < DIST_THRESHOLD) || (dR < DIST_THRESHOLD);

  if (!oaActive && !tooClose)
    return;
  if (!oaActive)
  {
    oaActive = true;
    stopAll();
    rotateCW();
  }
  else
  {
    if ((getDistance(TRIG_LEFT, ECHO_LEFT) >= DIST_THRESHOLD) &&
        (getDistance(TRIG_MIDDLE, ECHO_MIDDLE) >= DIST_THRESHOLD) &&
        (getDistance(TRIG_RIGHT, ECHO_RIGHT) >= DIST_THRESHOLD))
    {
      stopAll();
      oaActive = false;
    }
  }
}

// ────────────────────────────────────────────────
//  Setup
// ────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  int mPins[] = {PWM1, IN1_1, IN2_1, PWM2, IN1_2, IN2_2, PWM3, IN1_3, IN2_3, PWM4, IN1_4, IN2_4};
  for (int p : mPins)
    pinMode(p, OUTPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_MIDDLE, OUTPUT);
  pinMode(ECHO_MIDDLE, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  pinMode(L1, INPUT);
  pinMode(L2, INPUT);
  pinMode(L3, INPUT);
  pinMode(L4, INPUT);
  pinMode(L5, INPUT);

  Serial.println("Bottle-Seeker v3 ready");
}

// ────────────────────────────────────────────────
//  Main loop
// ────────────────────────────────────────────────
void loop()
{
  unsigned long now = millis();

  // —— LINE_SEARCH ———————————————————————————————
  if (mode == LINE_SEARCH)
  {
    ();
    if (oaActive)
    {
      // keep pivoting until clear
      return;
    }

    // read all 5 IR sensors
    bool s1 = analogRead(L1) / 1023.0 < threshold;
    bool s2 = analogRead(L2) / 1023.0 < threshold;
    bool s3 = analogRead(L3) / 1023.0 < threshold;
    bool s4 = analogRead(L4) / 1023.0 < threshold;
    bool s5 = analogRead(L5) / 1023.0 < threshold;

    switch (lsState)
    {
    case LS_ROLL:
      driveForward();
      if (s1 || s2 || s3 || s4 || s5)
      {
        // record which side saw black first
        if (s1 || s2)
          lsCounter = -1;
        else if (s4 || s5)
          lsCounter = +1;
        else
          lsCounter = -1;
        lsStamp = now;
        lsState = LS_BLAST;
        oaActive = false;
      }
      break;

    case LS_BLAST:
      if (now - lsStamp < BLAST_MS)
      {
        driveForward();
      }
      else
      {
        // begin pivot
        lsStamp = now;
        lsState = LS_PIVOT;
        if (lsCounter < 0)
          rotateCCW();
        else
          rotateCW();
      }
      break;

    case LS_PIVOT:
      if (now - lsStamp < PIVOT_MS)
      {
        // keep spinning
      }
      else
      {
        stopAll();
        lsState = LS_DONE;
      }
      break;

    case LS_DONE:
      driveForward();
      mode = LINE_FOLLOW;
      break;
    }
    return;
  }

  // —— LINE_FOLLOW ——————————————————————————————
  // ────────────────────────────────────────────────
  //  Inside your loop(), after handling LINE_SEARCH:
  // ────────────────────────────────────────────────

  // —— LINE_FOLLOW ——————————————————————————————
  if (mode == LINE_FOLLOW)
  {
    // 1) sample & normalize the five sensors
    float v1 = analogRead(L1) / 1023.0;
    float v2 = analogRead(L2) / 1023.0;
    float v3 = analogRead(L3) / 1023.0;
    float v4 = analogRead(L4) / 1023.0;
    float v5 = analogRead(L5) / 1023.0;

    bool s1 = v1 < threshold; // far left
    bool s2 = v2 < threshold; // left
    bool s3 = v3 < threshold; // center
    bool s4 = v4 < threshold; // right
    bool s5 = v5 < threshold; // far right

    // 2) If all see black → LOST LINE: stop & wait for any white
    if (s1 && s2 && s3 && s4 && s5)
    {
      stopAll();
      while (true)
      {
        float w1 = analogRead(L1) / 1023.0;
        float w2 = analogRead(L2) / 1023.0;
        float w3 = analogRead(L3) / 1023.0;
        float w4 = analogRead(L4) / 1023.0;
        float w5 = analogRead(L5) / 1023.0;
        if (w1 >= threshold || w2 >= threshold || w3 >= threshold || w4 >= threshold ||
            w5 >= threshold)
        {
          break;
        }
        delay(20);
      }
      // back on line → resume forward
      driveForward();
      return;
    }

    // 3) 90° turns if extreme sensor hits
    if (s1)
    {
      stopAll();
      rotateCCW();
      driveForward();
    }
    else if (s5)
    {
      stopAll();
      rotateCW();
      driveForward();
    }
    // 4) gentle steering corrections
    else if (s2)
    {
      gentleLeft();
    }
    else if (s4)
    {
      gentleRight();
    }
    // 5) straight ahead if center (or nothing) sees line
    else
    {
      driveForward();
    }

    delay(20); // maintain a 20 ms loop time
    return;
  }

  // —— fallback for any other mode —————————————————————————
  driveForward();
}