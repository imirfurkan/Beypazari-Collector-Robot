#include "BottleSeeker.h"
#include "Grippers.h"
#include "motors.h"
#include "line.h"
#include <NewPing.h>

enum RobotState
{
  SEEKING,
  GRIPPING,
  NAVIGATING
};
static RobotState robotState = SEEKING;

// ── configure your “auxiliary” sonar here ─────────────────
static constexpr unsigned int ISR_DIST_THRESHOLD = 100; // TODO walls
static constexpr int US_ISR_TRIG_PIN = 43;              // wire your extra HC-SR04 here
static constexpr int US_ISR_ECHO_PIN = 44;
static NewPing sonarISR(US_ISR_TRIG_PIN, US_ISR_ECHO_PIN, ISR_DIST_THRESHOLD);

// ── for coordinating between ISR and main loop ─────────────
volatile bool rotateRequested = false;

// this callback runs in the Timer2 interrupt context.
// Keep it super short: just test the distance and set a flag.
static void ISR_echoCallback()
{
  // duration (µs) from last ping:
  unsigned int uS = sonarISR.ping_timer();
  unsigned int cm = uS / US_ROUNDTRIP_CM;
  if (cm > 0 && cm <= ISR_DIST_THRESHOLD)
    rotateRequested = true;

  // schedule the next ping:
  sonarISR.ping_timer(ISR_echoCallback);
}

void setup()
{
  Serial.begin(115200);
  BottleSeeker_setup();
  Grippers_setup();
  Line_setup();
  sonarISR.ping_timer(ISR_echoCallback);
}

void loop()
{
  switch (robotState)
  {
  case SEEKING:
    if (BottleSeeker_loop())
    {
      robotState = GRIPPING;
    }
    break;

  case GRIPPING:
    if (Grippers_loop())
    {
      if (bottleRejected())
      {
        Motor_driveBackward();
        delay(1000); // TODO calibrate
        Motor_rotateCW();
        delay(1000); // TODO calibrate
        Motor_stopAll();
      }
      // in both reject and store cases go back to SEEKING
      robotState = NAVIGATING;
    }
    break;
  case NAVIGATING:
    if (Line_loop())
    {
      Serial.println(F(">> TARGET AREA REACHED! Placing bottles..."));
      Motor_driveForward();
      delay(1000); // TODO calibrate
      Motor_stopAll();
      // Grippers_placeBottle(); // TODO
      robotState = SEEKING;
    }
    break;
  }

  delay(10);
} // loop
