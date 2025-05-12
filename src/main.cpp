#include <Arduino.h>
#include "tof.h"
#include "bottleSeeker.h"
#include "grippers.h"
#include "motors.h"
#include "line.h"

// This flag lives in tof.cpp:
extern volatile bool triggeredObstacle;

// ── Robot State Machine ─────────────────────────────────
enum RobotState
{
  SEEKING,
  GRIPPING,
  NAVIGATING,
  INTERRUPT,
  COMPLETED
};
static RobotState robotState = SEEKING;
static RobotState previousState = SEEKING;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }

  setupTOF(); // all VL53L0X init + attachInterrupt(FALLING)
  BottleSeeker_setup();
  Grippers_setup();
  Line_setup();
}

void loop()
{
  // 1) Check TOF‐ISR flag first → jump to INTERRUPT state
  if (triggeredObstacle)
  {
    disableTOFInterrupt(); // mask until we’ve handled it
    previousState = robotState;
    robotState = INTERRUPT;
    triggeredObstacle = false;
  }

  // 2) Run your normal state machine
  switch (robotState)
  {
  case SEEKING:
    enableTOFInterrupt();
    if (BottleSeeker_loop())
    {
      disableTOFInterrupt();
      robotState = GRIPPING;
    }
    break;

  case GRIPPING:
    if (Grippers_loop())
    {
      if (bottleRejected())
      {
        Motor_driveBackward();
        delay(1000);
        Motor_rotateCW();
        delay(1000);
        Motor_stopAll();
      }
      if (bottleCount == 2)
        robotState = NAVIGATING;
      else
        robotState = SEEKING;
      enableTOFInterrupt();
    }
    break;

  case NAVIGATING:
    if (Line_loop())
    {
      Serial.println(F(">> TARGET AREA REACHED! Placing bottles..."));
      Motor_driveForward();
      delay(1000);
      Motor_stopAll();

      // Sequence for placing bottles
      Grippers_lowerElbows(); // Lower both elbows
      delay(700);

      Grippers_openClaws(); // Open both claws
      delay(700);

      Grippers_raiseElbows(); // Raise both elbows
      delay(700);

      Grippers_closeClaws(); // Close both claws
      delay(700);

      bottleCount = 0; // Reset bottle count

      Serial.println(F(">> Bottles placed successfully"));
      robotState = COMPLETED;
    }
    break;

  case INTERRUPT:
    Serial.println(F(" Wall (<80 mm) detected! Avoiding..."));
    Motor_rotateCW();
    delay(random(500, 701)); // tweak this for your turn angle
    Motor_stopAll();
    enableTOFInterrupt(); // re-arm the sensor ISR
    robotState = previousState;
    break;

  case COMPLETED:
    // Do nothing - robot stays in place
    // Optionally blink an LED or do something to indicate completion
    break;
  }
  delay(10);
}
