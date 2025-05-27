/////////////////////////////////////////////////////////////////////////////
/////////////////// EVERYTHING INCLUDED /////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

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
    clearTOFInterrupt();
    if (BottleSeeker_loop())
    {
      disableTOFInterrupt();
      robotState = GRIPPING;
      delay(1500);
    }
    break;

  case GRIPPING:
    if (Grippers_loop())
    {
      if (bottleRejected())
      {
        Motor_driveBackward();
        delay(2000);
        Motor_stopAll();
        delay(200);

        Motor_rotateCW();
        // static NewPing ultrasonicL(TRIG_LEFT, ECHO_LEFT, SEEKER_THRESHOLD_SIDES);
        // static NewPing ultrasonicM(TRIG_MIDDLE, ECHO_MIDDLE, SEEKER_THRESHOLD_MIDDLE);
        // static NewPing ultrasonicR(TRIG_RIGHT, ECHO_RIGHT, SEEKER_THRESHOLD_SIDES);

        // // start rotating CW until left ultrasonic sees nothing closer than threshold
        // // Motor_rotateCW();
        // while (readDistance(ultrasonicL) <= SEEKER_THRESHOLD_SIDES ||
        //        readDistance(ultrasonicM) <= SEEKER_THRESHOLD_MIDDLE ||
        //        readDistance(ultrasonicR) <= SEEKER_THRESHOLD_SIDES)
        // {
        //   // still “seeing” something on the left → keep spinning
        // }
        unsigned long extra = random(1000, 2000);
        delay(extra); // sol sensor okumayana kadar + belli bi sure
        Motor_stopAll();
      }
      if (bottleCount == 2)
        robotState = NAVIGATING;
      else
        robotState = SEEKING;
    }
    break;

  case NAVIGATING: // fix why the code doesn't enter this state
    // TODO ensure enable tof?
    if (Line_loop())
    {
      Serial.println(F(">> TARGET AREA REACHED! Placing bottles..."));
      Motor_driveBackward();
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
    clearTOFInterrupt();
    Serial.println(F(" Wall (<80 mm) detected! Avoiding..."));
    Motor_rotateCW();
    delay(random(1000, 1501)); // TODO tweak this for the turn angle
    Motor_stopAll();
    enableTOFInterrupt(); // re-arm the sensor ISR
    robotState = previousState;
    break;

  case COMPLETED:
    // Do nothing - robot stays in place
    // Optionally blink an LED or do something to indicate completion
    delay(10);
  }
}

// // // ///////////////////////////////////////////////////////////////////////////
// // // ///////////////// EVERYTHING W/O INTERRUPT ////////////////////////////////
// // // ///////////////////////////////////////////////////////////////////////////

// #include <Arduino.h>
// #include "bottleSeeker.h"
// #include "grippers.h"
// #include "motors.h"
// #include "line.h"

// // ── Robot State Machine ─────────────────────────────────
// enum RobotState
// {
//   SEEKING,
//   GRIPPING,
//   NAVIGATING,
//   COMPLETED
// };
// static RobotState robotState = SEEKING; // TODO na

// void setup()
// {
//   Serial.begin(115200);
//   while (!Serial)
//   {
//     delay(10);
//   }

//   // Initialize all modules
//   BottleSeeker_setup();
//   Grippers_setup();
//   Line_setup();
// }

// void loop()
// {
//   // 2) Run your normal state machine
//   switch (robotState)
//   {
//   case SEEKING:
//     if (Grippers_getBottleCount() == 2)
//     {
//       Serial.println(F(">> 2 bottles in gripper!"));
//       robotState = NAVIGATING;
//       Motor_driveForward(); // ← kick the wheels on every pass

//       // break;
//     }
//     if (BottleSeeker_loop())
//     {
//       robotState = GRIPPING;
//       delay(1500); // TODO: remove this delay
//     }
//     break;

//   case GRIPPING:
//     if (Grippers_loop())
//     {
//       if (bottleRejected())
//       {
//         Motor_driveBackward();
//         delay(2000); // TODO calibrate
//         Motor_stopAll();
//         delay(200);

//         Motor_rotateCW();
//         static NewPing ultrasonicL(TRIG_LEFT, ECHO_LEFT, SEEKER_THRESHOLD_SIDES);
//         static NewPing ultrasonicM(TRIG_MIDDLE, ECHO_MIDDLE, SEEKER_THRESHOLD_MIDDLE);
//         static NewPing ultrasonicR(TRIG_RIGHT, ECHO_RIGHT, SEEKER_THRESHOLD_SIDES);

//         // start rotating CW until left ultrasonic sees nothing closer than threshold
//         // Motor_rotateCW();
//         while (readDistance(ultrasonicL) <= SEEKER_THRESHOLD_SIDES ||
//                readDistance(ultrasonicM) <= SEEKER_THRESHOLD_MIDDLE ||
//                readDistance(ultrasonicR) <= SEEKER_THRESHOLD_SIDES)
//         {
//           // still “seeing” something on the left → keep spinning
//         }
//         unsigned long extra = random(200, 600);
//         delay(extra); // TODO sol sensor okumayana kadar + belli bi sure
//         Motor_stopAll();
//       }
//       if (Grippers_getBottleCount() == 2)
//       {
//         Serial.println(F(">> 2 bottles in gripper a!"));
//         robotState = NAVIGATING;
//         Motor_driveForward(); // ← kick the wheels on every pass
//         break;
//       }
//       else
//         robotState = SEEKING;
//     }
//     break;

//   case NAVIGATING:
//     if (Line_loop())
//     {
//       Serial.println(F(">> TARGET AREA REACHED! Placing bottles..."));
//       Motor_driveForward();
//       delay(1000);
//       Motor_stopAll();

//       // Sequence for placing bottles
//       Grippers_lowerElbows(); // Lower both elbows
//       delay(700);

//       Grippers_openClaws(); // Open both claws
//       delay(700);

//       Grippers_raiseElbows(); // Raise both elbows
//       delay(700);

//       Grippers_closeClaws(); // Close both claws
//       delay(700);

//       Serial.println(F(">> Bottles placed successfully"));
//       robotState = COMPLETED;
//     }
//     break;

//   case COMPLETED:
//     // Do nothing - robot stays in place Optionally blink an LED or
//     // do something to indicate completion break;
//     break;
//   }

//   delay(10);
// }