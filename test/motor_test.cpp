///////////////////////////////////////////////////////////////////////////////
///////////////////// MOTOR TEST /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include "motors.h"
#include "Grippers.h"
#include <Arduino.h>

// ── Setup: initialize pins and driver ──────────────────────
void setup()
{
  Grippers_setup();
  Motor_setup();
}

// ── Main loop: forward → CW → CCW, each for 5 s ───────────
void loop()
{
  // 1) Go straight ahead
  Motor_driveForward();
  delay(2000); // 5000 ms = 5 s

  // // 2) Spin in place clockwise
  // Motor_rotateCW();
  // delay(5000);

  // // 3) Spin in place counter-clockwise
  // Motor_rotateCCW();
  // delay(5000);

  // then immediately repeats…
}
