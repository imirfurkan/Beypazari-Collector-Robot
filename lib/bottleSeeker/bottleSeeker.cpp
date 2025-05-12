#include <NewPing.h>
#include "BottleSeeker.h"
#include "motors.h"
#include "ultrasonic.h"

// ────────────────────────────────────────────────
//  Behaviour parameters
// ────────────────────────────────────────────────
static constexpr unsigned int ROTATE_EXTRA_MS = 5000; // ms
static constexpr uint8_t FALLBACK_DEBOUNCE = 4;
static constexpr int BASE_SPEED_DEFAULT = 100;
static constexpr int BASE_SPEED_TURNING = 70;

// ────────────────────────────────────────────────
//  Internal state
// ────────────────────────────────────────────────
static bool rotating = false;     // are we inside that window?
static int8_t rotDir = 0;         //  ‑1 = CCW , +1 = CW
static unsigned long rotEnd = 0;  // millis() deadline
static uint8_t fallbackCount = 0; // require 5 consecutive “no‐sonar” passes

static NewPing sonarL(TRIG_LEFT, ECHO_LEFT, SEEKER_THRESHOLD);
static NewPing sonarM(TRIG_MIDDLE, ECHO_MIDDLE, SEEKER_THRESHOLD);
static NewPing sonarR(TRIG_RIGHT, ECHO_RIGHT, SEEKER_THRESHOLD);

enum BottleSeekerState
{
  SEARCH,
  APPROACH
};
static BottleSeekerState currentState = SEARCH;

// ────────────────────────────────────────────────
//  Public API
// ────────────────────────────────────────────────
void BottleSeeker_setup()
{
  // motor module handles TB6612 pins & standby
  Motor_setup();
  Motor_setBaseSpeed(BASE_SPEED_DEFAULT);

  // ultrasonic sensor pins
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_MIDDLE, OUTPUT);
  pinMode(ECHO_MIDDLE, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
}

// ────────────────────────────────────────────────
//  LOOP (simple FSM)
// ────────────────────────────────────────────────
bool BottleSeeker_loop()
{
  unsigned int dL = readDistance(sonarL);
  unsigned int dM = readDistance(sonarM);
  unsigned int dR = readDistance(sonarR);

  bool objL = (dL < SEEKER_THRESHOLD);
  bool objM = (dM < SEEKER_THRESHOLD);
  bool objR = (dR < SEEKER_THRESHOLD);
  bool objAny = (objL || objM || objR);
  bool objGrip = (dM < GRIPPER_THRESHOLD);

  switch (currentState)
  {

  case SEARCH:
    if (!objAny)
    {
      Serial.println(F("SEARCH: driving forward"));
      Motor_driveForward();
      // delay(50); // TODO remove this?
      // removing this delay so the ultrasonic sensors can be read more frequently
      return false;
    }
    Serial.println(F("SEARCH: object detected -> APPROACH"));
    currentState = APPROACH;
    break;

  case APPROACH:
    // within grip range?
    if (objGrip)
    {
      fallbackCount = 0;
      Serial.println(F("APPROACH: in grip range -> ready"));
      Motor_stopAll();
      currentState = SEARCH;
      return true;
    }

    // continue timed rotation
    if (rotating)
    {
      fallbackCount = 0;
      Serial.println(F("APPROACH: still rotating…"));
      if (rotDir < 0)
        Motor_rotateCCW();
      else
        Motor_rotateCW();
      bool centered = objM;
      if (centered || millis() >= rotEnd)
      {
        delay(500); // turn a little bit longer after the middle sensor is activated
        rotating = false;
        ensureMotorSpeed(BASE_SPEED_DEFAULT);
        Motor_stopAll();
        Serial.println(centered ? F("APPROACH: aligned -> forward")
                                : F("APPROACH: timeout -> fallback"));
        if (centered)
        {
          Motor_driveForward();
          // delay(200); // TODO remove this?
        }
      }
      return false;
    }

    // ── normal steering logic ──

    // Only-middle sees → drive straight
    if (!objL && objM && !objR)
    {
      fallbackCount = 0;
      ensureMotorSpeed(BASE_SPEED_DEFAULT);
      Serial.println(F("APPROACH: centre -> forward"));
      Motor_driveForward();
      return false;
    }
    // Left-side sees → start CCW spin
    else if ((objL && !objM && !objR) || (objL && objM && !objR))
    {
      fallbackCount = 0;
      ensureMotorSpeed(BASE_SPEED_TURNING);
      rotDir = -1;
      rotating = true; // <— THIS makes the top if(rotating) run next loop
      rotEnd = millis() + ROTATE_EXTRA_MS;
      Serial.println(F("APPROACH: start CCW"));
      Motor_rotateCCW();
      return false;
    }
    // Right-side sees → start CW spin
    else if ((!objL && !objM && objR) || (!objL && objM && objR))
    {
      fallbackCount = 0;
      ensureMotorSpeed(BASE_SPEED_TURNING);
      rotDir = +1;
      rotating = true; // <— THIS makes the top if(rotating) run next loop
      rotEnd = millis() + ROTATE_EXTRA_MS;
      Serial.println(F("APPROACH: start CW"));
      Motor_rotateCW();
      return false;
    }
    // TODO Edge cases → forward for now
    else if ((objL && !objM && objR) || (objL && objM && objR))
    {
      fallbackCount = 0;
      ensureMotorSpeed(BASE_SPEED_DEFAULT);
      Serial.println(F("APPROACH: both-sides → driveForward"));
      Motor_driveForward();
      return false;
    }
    else
    {
      fallbackCount++;
      if (fallbackCount >= FALLBACK_DEBOUNCE)
      {
        currentState = SEARCH;
        fallbackCount = 0;
        Serial.println(F("APPROACH: fallback -> SEARCH (debounced)"));
      }
      else
      {
        Serial.print(F("APPROACH: fallback debounce #"));
        Serial.println(fallbackCount);
      }
      return false;
    }
  } // end of switch

  return false;
} // end of loop

// // test code
// #include <Arduino.h>
// #include "BottleSeeker.h"
// #include "motors.h"

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) { /* wait for Serial */ }

//   BottleSeeker_setup();
//   Serial.println(F("== BottleSeeker TEST START =="));
// }

// void loop() {
//   // Run one iteration of the FSM
//   bool found = BottleSeeker_loop();

//   if (found) {
//     Serial.println(F(">>> BOTTLE IN GRIP RANGE!"));

//     // Hold here until the seeker resets back to SEARCH
//     while (BottleSeeker_loop()) {
//       delay(10);
//     }
//     Serial.println(F("-- seeker reset to SEARCH --"));
//   }

//   // Small delay to avoid flooding the Serial output
//   delay(10);
// }
