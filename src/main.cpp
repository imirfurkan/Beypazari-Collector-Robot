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

// ///////////////////////////////////////////////////////////////////////////
// ///////////////// EVERYTHING W/O INTERRUPT ////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////

// // #include <Arduino.h>
// // #include "bottleSeeker.h"
// // #include "grippers.h"
// // #include "motors.h"
// // #include "line.h"

// // // ── Robot State Machine ─────────────────────────────────
// // enum RobotState
// // {
// //   SEEKING,
// //   GRIPPING,
// //   NAVIGATING,
// //   COMPLETED
// // };
// // static RobotState robotState = SEEKING; // TODO na

// // void setup()
// // {
// //   Serial.begin(115200);
// //   while (!Serial)
// //   {
// //     delay(10);
// //   }

// //   // Initialize all modules
// //   BottleSeeker_setup();
// //   Grippers_setup();
// //   Line_setup();
// // }

// // void loop()
// // {
// //   // 2) Run your normal state machine
// //   switch (robotState)
// //   {
// //   case SEEKING:
// //     if (BottleSeeker_loop())
// //     {
// //       robotState = GRIPPING;
// //       delay(1500); // TODO: remove this delay
// //     }
// //     break;

// //   case GRIPPING:
// //     if (Grippers_loop())
// //     {
// //       if (bottleRejected())
// //       {
// //         Motor_driveBackward();
// //         delay(1000);
// //         Motor_stopAll();
// //         delay(200);

// //         static NewPing ultrasonicL(TRIG_LEFT, ECHO_LEFT, SEEKER_THRESHOLD_SIDES);
// //         static NewPing ultrasonicM(TRIG_MIDDLE, ECHO_MIDDLE, SEEKER_THRESHOLD_MIDDLE);
// //         static NewPing ultrasonicR(TRIG_RIGHT, ECHO_RIGHT, SEEKER_THRESHOLD_SIDES);

// //         // start rotating CW until left ultrasonic sees nothing closer than threshold
// //         Motor_rotateCW();
// //         while (readDistance(ultrasonicL) <= SEEKER_THRESHOLD_SIDES ||
// //                readDistance(ultrasonicM) <= SEEKER_THRESHOLD_MIDDLE ||
// //                readDistance(ultrasonicR) <= SEEKER_THRESHOLD_SIDES)
// //         {
// //           // still “seeing” something on the left → keep spinning
// //         }
// //         unsigned long extra = random(200, 600);
// //         delay(extra); // TODO sol sensor okumayana kadar + belli bi sure
// //         Motor_stopAll();
// //       }
// //       if (bottleCount == 2)
// //         robotState = NAVIGATING;
// //       else
// //         robotState = SEEKING;
// //     }
// //     break;

// //   case NAVIGATING:
// //     if (Line_loop())
// //     {
// //       Serial.println(F(">> TARGET AREA REACHED! Placing bottles..."));
// //       Motor_driveForward();
// //       delay(1000);
// //       Motor_stopAll();

// //       // // Sequence for placing bottles
// //       // Grippers_lowerElbows(); // Lower both elbows
// //       // delay(700);

// //       // Grippers_openClaws(); // Open both claws
// //       // delay(700);

// //       // Grippers_raiseElbows(); // Raise both elbows
// //       // delay(700);

// //       // Grippers_closeClaws(); // Close both claws
// //       // delay(700);

// //       bottleCount = 0; // Reset bottle count

// //       Serial.println(F(">> Bottles placed successfully"));
// //       robotState = COMPLETED;
// //     }
// //     break;

// //   case COMPLETED:
// //     // Do nothing - robot stays in place
// //     // Optionally blink an LED or do something to indicate completion
// //     break;
// //   }

// //   delay(10);
// // }

// ///////////////////////////////////////////////////////////////////////////////
// ///////////////////// MOTOR TEST /////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////

// // #include "motors.h"

// //     // ── Setup: initialize pins and driver ──────────────────────
// //     void
// //     setup()
// // {
// //   Motor_setup();
// // }

// // // ── Main loop: forward → CW → CCW, each for 5 s ───────────
// // void loop()
// // {
// //   // 1) Go straight ahead
// //   Motor_driveForward();
// //   delay(2000); // 5000 ms = 5 s

// //   // 2) Spin in place clockwise
// //   Motor_rotateCW();
// //   delay(5000);

// //   // 3) Spin in place counter-clockwise
// //   Motor_rotateCCW();
// //   delay(5000);

// //   // then immediately repeats…
// // }

// // ///////////////////////////////////////////////////////////////////////////////
// // ///////////////////// GRIPPER TEST ///////////////////////////////////////////
// // ///////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "Grippers.h"

void setup()
{
  // start serial for debug (optional)
  Serial.begin(115200);
  while (!Serial)
  { /* wait for Serial on some boards */
  }

  // initialize gripper hardware and state
  Grippers_setup();
}

void loop()
{
  // call the gripper state-machine; if it's still mid-cycle, bail out immediately
  if (!Grippers_loop())
  {
    return;
  }

  // ← here only when Grippers_loop() has returned true
  Serial.println(F("Gripper cycle complete!"));
}

// ///////////////////////////////////////////////////////////////////////////////
// ///////////////////// BOTTLE SEEKER TETS /////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////

// // #include <Arduino.h>
// // #include "BottleSeeker.h"
// // #include "motors.h"

// // void setup()
// // {
// //   Serial.begin(115200);
// //   while (!Serial)
// //   { /* wait for Serial */
// //   }

// //   BottleSeeker_setup();
// //   Serial.println(F("== BottleSeeker TEST START =="));
// // }

// // void loop()
// // {
// //   // Run one iteration of the FSM
// //   bool found = BottleSeeker_loop();

// //   if (found)
// //   {
// //     Serial.println(F(">>> BOTTLE IN GRIP RANGE!"));

// //     // Hold here until the seeker resets back to SEARCH
// //     while (BottleSeeker_loop())
// //     {
// //       delay(10);
// //     }
// //     Serial.println(F("-- seeker reset to SEARCH --"));
// //   }

// //   // Small delay to avoid flooding the Serial output
// //   delay(10);
// // }

// // #include <Arduino.h>
// // #include <Servo.h>

// // // from grippers.cpp
// // const uint8_t CAP_PUSHER_PIN = 40;
// // const uint8_t CAP_UP_ANGLE = 90;    // TODO
// // const uint8_t CAP_DOWN_ANGLE = 178; // TODO

// // // your switch pin
// // const uint8_t SWITCH_PIN = 41;

// // Servo capSrv;

// // void setup()
// // {
// //   Serial.begin(115200);
// //   pinMode(SWITCH_PIN, INPUT_PULLUP);
// //   capSrv.attach(CAP_PUSHER_PIN);
// // }

// // void loop()
// // {
// //   Serial.println(F("TEST_CAP → pressing & testing cap")); // ← tweaked message

// //   // 1) Drop the cap-pusher
// //   capSrv.attach(CAP_PUSHER_PIN);
// //   capSrv.write(CAP_DOWN_ANGLE);
// //   delay(700);

// //   // 2) POLL the microswitch
// //   if (digitalRead(SWITCH_PIN) == LOW)
// //   {
// //     Serial.println(F("→ Cap detected")); // switch closed
// //   }
// //   else
// //   {
// //     Serial.println(F("→ No cap detected")); // switch open
// //   }

// //   // 3) Retract the pusher
// //   capSrv.write(CAP_UP_ANGLE);
// //   delay(250);
// //   capSrv.detach();

// //   // 4) Pause before repeating
// //   delay(2000);
// // }