#include <Arduino.h>

// LINE SEARCH & FOLLOW
// #TODO
// SISE SAYISINA GORE BLAST MS DEGISECEK

// ────────────────────────────────────────────────
//  Pin map (Mega) – unchanged **physical** wiring
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
const int L1 = A4, L2 = A3, L3 = A2, L4 = A1, L5 = A0;

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
const unsigned int DIST_THRESHOLD = 10; // cm, for obstacle‐avoid in LINE_SEARCH
int baseSpeed = 100;                    // can change dynamically
const int BASE_SPEED_DEFAULT = 100;
const int BASE_SPEED_PIVOT = 50;
const int BASE_SPEED_GENTLE = 50;

float K_unloaded[4] = {0.974, 0.808, 0.835, 0.750};
const bool forwardDir = true;

const float steerPct = 0.50;               // ±15% gentle steer
const float threshold = 0.15;              // normalized IR threshold
const unsigned long BLAST_MS = 500;        // “blast” forward after first line hit
const unsigned long CORNER_BLAST_MS = 300; // short-blast before pivot
const unsigned long PIVOT_MS = 1000;       // open‐loop pivot

// ────────────────────────────────────────────────
//  State machines
// ────────────────────────────────────────────────
enum Robot_State
{
  LINE_SEARCH,
  LINE_FOLLOW
};

Robot_State robotState = LINE_SEARCH;

enum LSState
{
  LS_ROLL,
  LS_BLAST,
  LS_PIVOT
};
LSState lsState = LS_ROLL;

// for LS timing and side flag
unsigned long lsStamp = 0;
int lsDirection = 0; // –1 = left‐first, +1 = right‐first

// obstacle‐avoid non-blocking
unsigned long lastObsChk = 0;
const unsigned long OBS_DT = 50;
bool oaActive = false;
bool tapeFound = false;
bool centerSeen = false;
bool pivotStarted = false;

// ────────────────────────────────────────────────
//  Hard‐turn flags & two‐phase state
// ────────────────────────────────────────────────
static bool hardLeftActive = false;
static unsigned long hardLeftStamp = 0;
static bool hardLeftPhase2 = false;

static bool hardRightActive = false;
static unsigned long hardRightStamp = 0;
static bool hardRightPhase2 = false;

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

void rotateCCW()
{
  driveMotor(0, PWM1, IN1_1, IN2_1, !forwardDir);
  driveMotor(1, PWM2, IN1_2, IN2_2, !forwardDir);
  driveMotor(2, PWM3, IN1_3, IN2_3, forwardDir);
  driveMotor(3, PWM4, IN1_4, IN2_4, forwardDir);
}

void rotateCW()
{
  driveMotor(0, PWM1, IN1_1, IN2_1, forwardDir);
  driveMotor(1, PWM2, IN1_2, IN2_2, forwardDir);
  driveMotor(2, PWM3, IN1_3, IN2_3, !forwardDir);
  driveMotor(3, PWM4, IN1_4, IN2_4, !forwardDir);
}

void gentleLeft()
{
  int leftPWM = int(BASE_SPEED_GENTLE * (1.0 - steerPct));
  int rightPWM = BASE_SPEED_GENTLE;
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

void gentleRight()
{
  int leftPWM = BASE_SPEED_GENTLE;
  int rightPWM = int(BASE_SPEED_GENTLE * (1.0 - steerPct));
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
  return us ? us / 58 : 999;
}

// ────────────────────────────────────────────────
//  Obstacle‐avoid (LINE_SEARCH only)
// ────────────────────────────────────────────────
void updateObstacleAvoid()
{
  if (robotState != LINE_SEARCH)
    return;
  unsigned long now = millis();
  if (now - lastObsChk < OBS_DT)
    return;
  lastObsChk = now;

  unsigned int dL = getDistance(TRIG_LEFT, ECHO_LEFT);
  unsigned int dM = getDistance(TRIG_MIDDLE, ECHO_MIDDLE);
  unsigned int dR = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  bool tooClose = (dL < DIST_THRESHOLD) || (dM < DIST_THRESHOLD) || (dR < DIST_THRESHOLD);

  // — debug —
  Serial.print("OA: dL=");
  Serial.print(dL);
  Serial.print("  dM=");
  Serial.print(dM);
  Serial.print("  dR=");
  Serial.print(dR);
  Serial.print("  tooClose=");
  Serial.println(tooClose);

  if (!oaActive && !tooClose)
    return;
  if (!oaActive)
  {
    oaActive = true;
    stopAll();
    delay(10);
    Serial.println("Obstacle detected → rotating CW");
    rotateCW();
    delay(500);
  }
  else if (dL >= DIST_THRESHOLD && dM >= DIST_THRESHOLD && dR >= DIST_THRESHOLD)
  {
    stopAll();
    Serial.println("Obstacle cleared → stop and delay");
    oaActive = false;
    Serial.println("devamke");
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

  Serial.println("Geypazari toplayici hazir");
}

// ────────────────────────────────────────────────
//  Main loop
// ────────────────────────────────────────────────
void loop()
{
  unsigned long now = millis();

  // — debug dump at top of loop —
  Serial.print("LOOP: State=");
  Serial.print(robotState == LINE_SEARCH ? "SEARCH" : "FOLLOW");
  Serial.print("  tapeFound=");
  Serial.print(tapeFound);
  Serial.print("  oaActive=");
  Serial.print(oaActive);
  Serial.print("  hardLeft=");
  Serial.print(hardLeftActive);
  Serial.print("  hardRight=");
  Serial.println(hardRightActive);

  // —— LINE_SEARCH ———————————————————————————————
  if (robotState == LINE_SEARCH)
  {
    if (!tapeFound)
    {
      updateObstacleAvoid();
      if (oaActive)
      {
        // keep pivoting until clear
        return;
      }
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
      Serial.println("LS_ROLL");
      driveForward();
      if (s1 || s2 || s3 || s4 || s5)
      {
        // record which side saw black first and assign directions
        if (s1 || s2)
          lsDirection = -1;
        else if (s4 || s5)
          lsDirection = +1;
        else
          lsDirection = -1;
        lsStamp = now;
        lsState = LS_BLAST;
        tapeFound = true;
        oaActive = false; // just in case
      }
      break;

    case LS_BLAST:
      Serial.println("LS_BLAST");
      if (!centerSeen)
      {
        // move forward until center sensor is triggered
        bool s3 = analogRead(L3) / 1023.0 < threshold;
        driveForward();
        if (s3)
        {
          centerSeen = true;
          lsStamp = now; // start blast timer *after* s3 is seen
        }
      }
      else
      {
        // once center seen, continue forward for BLAST_MS
        if (now - lsStamp < BLAST_MS)
        {
          driveForward();
        }
        else
        {
          // done blasting, begin pivot
          stopAll();
          lsStamp = now;
          lsState = LS_PIVOT;
          pivotStarted = false;
        }
      }
      break;

    case LS_PIVOT:
      Serial.println("LS_PIVOT");
      if (!pivotStarted)
      {
        pivotStarted = true;
        baseSpeed = BASE_SPEED_PIVOT; // slow down the speed for the pivot
        lsStamp = now;                // start timing here
        if (lsDirection < 0)
          rotateCW();
        else
          rotateCCW();
      }

      bool s3 = analogRead(L3) / 1023.0 < threshold;

      if (s3 || (now - lsStamp >= PIVOT_MS)) // if s3 sees, or a pivot_ms time has passed (to avoid
                                             // infinite pivot just in case)
      {
        stopAll();
        baseSpeed = BASE_SPEED_DEFAULT; // return to the normal speed for line follow
        driveForward();
        robotState = LINE_FOLLOW;
      }
      break;
    }
    return;
  }

  // —— LINE_FOLLOW ——————————————————————————————
  if (robotState == LINE_FOLLOW)
  {
    // sample & normalize sensors
    float v1 = analogRead(L1) / 1023.0;
    float v2 = analogRead(L2) / 1023.0;
    float v3 = analogRead(L3) / 1023.0;
    float v4 = analogRead(L4) / 1023.0;
    float v5 = analogRead(L5) / 1023.0;

    bool s1 = v1 < threshold, s2 = v2 < threshold;
    bool s3 = v3 < threshold, s4 = v4 < threshold;
    bool s5 = v5 < threshold;

    // — if we're mid‐hard left, finish it —
    if (hardLeftActive)
    {
      if (!hardLeftPhase2)
      {
        if (hardLeftStamp == 0)
          hardLeftStamp = now;
        if (now - hardLeftStamp < CORNER_BLAST_MS)
        {
          driveForward();
        }
        else
        {
          hardLeftPhase2 = true;
          stopAll();
          delay(50);
        }
      }
      else // Phase 2: turning
      {
        bool _s1 = analogRead(L1) / 1023.0 < threshold;
        bool _s3 = analogRead(L3) / 1023.0 < threshold;
        bool _s5 = analogRead(L5) / 1023.0 < threshold;
        if (!_s1 && _s3 && !_s5) // if we have centered the line
        {
          stopAll();
          delay(50);
          driveForward();
          hardLeftActive = false;
          hardLeftPhase2 = false;
          hardLeftStamp = 0;
        }
        else
        {
          rotateCCW(); // rotate until centered
        }
      }
      return;
    }

    // — if we're mid‐hard right, finish it —
    if (hardRightActive)
    {
      if (!hardRightPhase2)
      {
        if (hardRightStamp == 0)
          hardRightStamp = now;
        if (now - hardRightStamp < CORNER_BLAST_MS)
        {
          driveForward();
        }
        else
        {
          hardRightPhase2 = true;
          stopAll();
          delay(50);
        }
      }
      else
      {
        bool _s1 = analogRead(L1) / 1023.0 < threshold;
        bool _s3 = analogRead(L3) / 1023.0 < threshold;
        bool _s5 = analogRead(L5) / 1023.0 < threshold;
        if (!_s5 && _s3 && !_s1)
        {
          stopAll();
          delay(50);
          driveForward();
          hardRightActive = false;
          hardRightPhase2 = false;
          hardRightStamp = 0;
        }
        else
        {
          rotateCW();
        }
      }
      return;
    }

    // — detect fresh hard‐left entry —
    if ((s1 && s2 && !s5) || (s1 && s2 && s3 && !s5))
    {
      Serial.println("DETECT: fresh HARD LEFT");
      hardLeftActive = true;
      hardLeftPhase2 = false;
      hardLeftStamp = now;
      driveForward();
      return;
    }
    // — detect fresh hard‐right entry —
    else if ((!s1 && s4 && s5) || (!s1 && s3 && s4 && s5))
    {
      Serial.println("DETECT: fresh HARD RIGHT");
      hardRightActive = true;
      hardRightPhase2 = false;
      hardRightStamp = now;
      driveForward();
      return;
    }
    // — gentle steering corrections —
    else if (!s1 && s2)
    {
      gentleLeft();
    }
    else if (s4 && !s5)
    {
      gentleRight();
    }
    // — default: straight ahead —
    else
    {
      driveForward();
    }

    delay(20);
    return;
  }
}