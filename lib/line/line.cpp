#include "line.h"
#include "ultrasonic.h" // provides TRIG_*, ECHO_*, DIST_THRESHOLD & readDistance()
#include "motors.h"
#include <Arduino.h>
#include <NewPing.h> // NewPing is brought in by ultrasonic.h, but include for clarity

// ────────────────────────────────────────────────
//  NewPing objects for obstacle‐avoid
// ────────────────────────────────────────────────
static NewPing sonarL(TRIG_LEFT, ECHO_LEFT, OBSTACLE_THRESHOLD);
static NewPing sonarM(TRIG_MIDDLE, ECHO_MIDDLE, OBSTACLE_THRESHOLD);
static NewPing sonarR(TRIG_RIGHT, ECHO_RIGHT, OBSTACLE_THRESHOLD);

// ────────────────────────────────────────────────
//  Parameters & State
// ────────────────────────────────────────────────

// IR sensorIR_THRESHOLD (normalized)
static constexpr float IR_THRESHOLD = 0.15f;

// Obstacle-avoid (NewPing range & debounce)
static constexpr unsigned long OBS_DT = 50; // ms

// Line-search timing
static constexpr unsigned long BLAST_MS = 700;            // after center hit
static constexpr unsigned long MAX_CORNER_BLAST_MS = 200; // prep before pivot
static constexpr unsigned long PIVOT_MS = 8000;           // max pivot time

// ── FSM States ─────────────────────────────────────────
enum LineState
{
  LINE_SEARCH,
  LINE_FOLLOW
};
static LineState lineState = LINE_FOLLOW;

enum LSState
{
  LS_ROLL,
  LS_BLAST,
  LS_PIVOT
};
static LSState lsState = LS_ROLL;

// ── Dynamic flags & timers ────────────────────────────
static bool oaActive = false;     // obstacle-avoid active?
static bool tapeFound = false;    // first detection in SEARCH?
static bool centerSeen = false;   // blast phase helper
static bool pivotStarted = false; // pivot phase helper

static unsigned long lastObsChk = 0; // for OBS_DT debounce
static unsigned long lsStamp = 0;    // for BLAST_MS & PIVOT_MS
static int lsDirection = 0;          // –1=left-first, +1=right-first

// ── Hard-turn flags & timers ─────────────────────────
static bool hardLeftActive = false;
static bool hardLeftPhase2 = false;
static unsigned long hardLeftStamp = 0;

static bool hardRightActive = false;
static bool hardRightPhase2 = false;
static unsigned long hardRightStamp = 0;

// ────────────────────────────────────────────────
//  Obstacle‐avoid (LINE_SEARCH only)
// ────────────────────────────────────────────────
void updateObstacleAvoid()
{
  if (lineState != LINE_SEARCH) // works only in line search
    return;
  unsigned long now = millis();
  if (now - lastObsChk < OBS_DT) // check every OBS_DT ms
    return;
  lastObsChk = now;

  unsigned int dL = readDistance(sonarL);
  unsigned int dM = readDistance(sonarM);
  unsigned int dR = readDistance(sonarR);

  bool tooClose =
      (dL < OBSTACLE_THRESHOLD) || (dM < OBSTACLE_THRESHOLD) || (dR < OBSTACLE_THRESHOLD);

  // // — debug —
  // Serial.print("OA: dL=");
  // Serial.print(dL);
  // Serial.print("  dM=");
  // Serial.print(dM);
  // Serial.print("  dR=");
  // Serial.print(dR);
  // Serial.print("  tooClose=");
  // Serial.println(tooClose);

  if (!oaActive && !tooClose)
    return;
  if (!oaActive) // it is tooClose, activate OA
  {
    oaActive = true;
    Motor_stopAll();
    delay(10);
    Serial.println("Obstacle detected → rotating CW");
    Motor_rotateCW();
    delay(500);
  }
  else if (dL >= OBSTACLE_THRESHOLD && dM >= OBSTACLE_THRESHOLD && dR >= OBSTACLE_THRESHOLD)
  {
    Motor_stopAll();
    Serial.println("Obstacle cleared → stop and delay");
    oaActive = false;
    Serial.println("devamke");
  }
}

// ────────────────────────────────────────────────
//  Setup
// ────────────────────────────────────────────────
void Line_setup()
{
  Serial.begin(115200);
  Motor_setup();
  // IR sensor pins
  pinMode(L0, INPUT);
  pinMode(L1, INPUT);
  pinMode(L2, INPUT);
  pinMode(L3, INPUT);
  pinMode(L4, INPUT);
  pinMode(L5, INPUT);
  Serial.println(F("Line-follower ready"));
}

// ────────────────────────────────────────────────
//  Main loop
// ────────────────────────────────────────────────
/// Runs one iteration; return falses true if the line was lost in FOLLOW (i.e. you hit the target).
bool Line_loop()
{
  unsigned long now = millis();

  // // — debug dump at top of loop —
  // Serial.print("LOOP: State=");
  // Serial.print(lineState == LINE_SEARCH ? "SEARCH" : "FOLLOW");
  // Serial.print("  tapeFound=");
  // Serial.print(tapeFound);
  // Serial.print("  oaActive=");
  // Serial.print(oaActive);
  // Serial.print("  hardLeft=");
  // Serial.print(hardLeftActive);
  // Serial.print("  hardRight=");
  // Serial.println(hardRightActive);

  // —— LINE_SEARCH ———————————————————————————————
  if (lineState == LINE_SEARCH)
  {
    if (!tapeFound)
    {
      updateObstacleAvoid();
      if (oaActive)
      {
        // keep pivoting until clear
        return false;
      }
    }
    // read all 5 IR sensors
    // bool s0 = analogRead(L0) / 1023.0f < IR_THRESHOLD;
    bool s1 = analogRead(L1) / 1023.0f < IR_THRESHOLD;
    bool s2 = analogRead(L2) / 1023.0f < IR_THRESHOLD;
    bool s3 = analogRead(L3) / 1023.0f < IR_THRESHOLD;
    bool s4 = analogRead(L4) / 1023.0f < IR_THRESHOLD;
    bool s5 = analogRead(L5) / 1023.0f < IR_THRESHOLD;

    Serial.print("L0=");
    Serial.print(analogRead(L0));
    Serial.print(" ");

    switch (lsState)
    {
    case LS_ROLL:
      // Serial.println("LS_ROLL");
      Motor_driveForward();
      // if (s0)
      // {
      //   lsStamp = now;
      //   lsState = LS_PIVOT;
      //   tapeFound = true;
      //   oaActive = false; // just in case
      // }
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
        oaActive = false;                      // just in case
        Motor_setBaseSpeed(MOTOR_PIVOT_SPEED); // slow down the speed
      }
      return false;

    case LS_BLAST:
      Serial.println("LS_BLAST");
      if (!centerSeen)
      {
        // move forward until center sensor is triggered
        bool s3 = analogRead(L3) / 1023.0 < IR_THRESHOLD;
        Motor_driveForward();
        if (s3)
        {
          centerSeen = true;
          lsStamp = now; // start blast timer *after* s3 is seen
        }
      }
      else
      {
        // once center seen, continue forward until the s0 (center of the car) reads black or a
        // BLAST_MS has passed
        if ((now - lsStamp < BLAST_MS)) // TODO buraya girmiyor s0 kısmını sildim
        {
          Serial.println("drive backward - line 219");
          Motor_driveBackward();
        }
        else
        {
          // done blasting, begin pivot
          Serial.println("begin pivot");
          Motor_stopAll();
          lsStamp = now;
          lsState = LS_PIVOT;
          pivotStarted = false;
        }
      }
      return false;

    case LS_PIVOT:
      Serial.println("LS_PIVOT");
      if (!pivotStarted)
      {
        pivotStarted = true;
        Motor_setBaseSpeed(MOTOR_PIVOT_SPEED); // slow down the speed for the pivot (already slowed
                                               // down in LS_BLAST)
        lsStamp = now;                         // start timing here
        if (lsDirection < 0)
        {
          Serial.println("rotating CCW");
          Motor_rotateCCW(); // TODO doğru mu
          // delay(2500);
          lsStamp = now; // start timing here
          return false;
        }
        else
        {
          Serial.println("rotating CW");
          Motor_rotateCW();
          // delay(2500);
          lsStamp = now; // start timing here
          return false;
        }
        // Motor_rotateCW();
      }

      bool s3 = analogRead(L3) / 1023.0 < IR_THRESHOLD;

      if (s3 || (now - lsStamp >= PIVOT_MS)) // if s3 sees, or a pivot_ms time has passed (to
                                             // avoid infinite pivot just in case)
      {
        Motor_stopAll();
        Motor_setBaseSpeed(
            MOTOR_SPEED_DEFAULT); // return false to the normal speed for line follow //TODO rotate
                                  // ten sonra direkt bunu aldigi icin geri gitmeye devam ediyor
        Serial.println("drive backward - line 262");
        Motor_driveBackward();
        lineState = LINE_FOLLOW;
      }
      return false;
    }
    return false;
  }

  // —— LINE_FOLLOW ——————————————————————————————
  if (lineState == LINE_FOLLOW)
  {

    // sample & normalize sensors
    bool s0 = analogRead(L0) / 1023.0f < IR_THRESHOLD;
    bool s1 = analogRead(L1) / 1023.0f < IR_THRESHOLD;
    bool s2 = analogRead(L2) / 1023.0f < IR_THRESHOLD;
    bool s3 = analogRead(L3) / 1023.0f < IR_THRESHOLD;
    bool s4 = analogRead(L4) / 1023.0f < IR_THRESHOLD;
    bool s5 = analogRead(L5) / 1023.0f < IR_THRESHOLD;

    // // —— Non-blocking target-area detection ——
    // bool anySee = (s1 || s2 || s3 || s4 || s5);
    // if (!anySee)
    // {
    //   if (!noLineDetected)
    //   {
    //     noLineDetected = true;
    //     noLineStamp = now;
    //   }
    //   else if (noLineDetected && (now - noLineStamp >= TARGET_MS))
    //   {
    //     // target reached
    //     noLineDetected = false;
    //     noLineStamp = 0;
    //     return true;
    //   }
    // }
    // else
    // {
    //   // reset timer on any sighting
    //   noLineDetected = false;
    //   noLineStamp = now; // just reset
    // }

    // // target‐area detection: no sensor sees the line → lost it
    // if (!(s0 || s1 || s2 || s3 || s4 || s5))
    // {
    //   // reset for next SEARCH if you want
    //   lineState = LINE_SEARCH;
    //   tapeFound = false;
    //   centerSeen = false;
    //   pivotStarted = false;
    //   // return true to notify main() that target is reached
    //   return true;
    // }

    // — if we're mid‐hard left, finish it —
    if (hardLeftActive)
    {
      if (!hardLeftPhase2)
      {
        if (hardLeftStamp == 0)
          hardLeftStamp = now;
        if ((s0) || (millis() - hardLeftStamp < MAX_CORNER_BLAST_MS)) // until s0 doesn't read.
        // TODO could be dangerous if s0 reading is not reliable at the center
        {
          Motor_driveBackward();
        }
        else
        {
          hardLeftPhase2 = true;
          Motor_stopAll();
          delay(50);
        }
      }
      else // Phase 2: turning
      {
        bool _s1 = analogRead(L1) / 1023.0f < IR_THRESHOLD;
        bool _s3 = analogRead(L3) / 1023.0f < IR_THRESHOLD;
        bool _s5 = analogRead(L5) / 1023.0f < IR_THRESHOLD;
        if (!_s1 && _s3 && !_s5) // if we have centered the line
        {
          Motor_stopAll();
          delay(50);
          Motor_driveBackward();
          hardLeftActive = false;
          hardLeftPhase2 = false;
          hardLeftStamp = 0;
        }
        else
        {
          Motor_rotateCCW(); // rotate until centered
        }
      }
      return false;
    }

    // — if we're mid‐hard right, finish it —
    if (hardRightActive)
    {
      if (!hardRightPhase2)
      {
        if (hardRightStamp == 0)
          hardRightStamp = now;
        if ((s0) || (millis() - hardRightStamp < MAX_CORNER_BLAST_MS))
        {
          Motor_driveBackward();
        }
        else
        {
          hardRightPhase2 = true;
          Motor_stopAll();
          delay(50);
        }
      }
      else
      {
        bool _s1 = analogRead(L1) / 1023.0f < IR_THRESHOLD;
        bool _s3 = analogRead(L3) / 1023.0f < IR_THRESHOLD;
        bool _s5 = analogRead(L5) / 1023.0f < IR_THRESHOLD;
        if (!_s5 && _s3 && !_s1)
        {
          Motor_stopAll();
          delay(50);
          Motor_driveBackward();
          hardRightActive = false;
          hardRightPhase2 = false;
          hardRightStamp = 0;
        }
        else
        {
          Motor_rotateCW(); // Motor_rotateCW();
        }
      }
      return false;
    }

    // — detect fresh hard‐left entry —
    if ((s1 && s2 && !s5) || (s1 && s2 && s3 && !s5))
    {
      Serial.println("DETECT: fresh HARD LEFT");
      hardLeftActive = true;
      hardLeftPhase2 = false;
      hardLeftStamp = now;
      Motor_driveBackward();
      return false;
    }
    // — detect fresh hard‐right entry —
    else if ((!s1 && s4 && s5) || (!s1 && s3 && s4 && s5))
    {
      Serial.println("DETECT: fresh HARD RIGHT");
      hardRightActive = true;
      hardRightPhase2 = false;
      hardRightStamp = now;
      Motor_driveBackward();
      return false;
    }
    // — gentle steering corrections —
    else if (!s1 && s2)
    {
      Motor_gentleLeft(); // Motor_gentleLeft();
      Serial.println("gentle left");
    }
    else if (s4 && !s5)
    {
      Motor_gentleRight(); // Motor_gentleRight();
      Serial.println("gentle right");
    }

    else if ((s1 && !s2 && !s3))
    {
      Motor_Left(); // Motor_gentleLeft();
      Serial.println("gentle left");
    }
    else if ((!s3 && !s4 && s5))
    {
      Motor_Right(); // Motor_gentleRight();
      Serial.println("gentle right");
    }
    // — default: straight ahead —
    else
    {
      Motor_driveBackward(); // TODO
    }

    delay(20);
    return false;
  }
  return false;
}