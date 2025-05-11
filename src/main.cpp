#include "BottleSeeker.h"
#include "Grippers.h"
#include "motors.h"
#include "line.h"

enum RobotState
{
  SEEKING,
  GRIPPING,
  NAVIGATING,
  INTERRUPT
};
static RobotState robotState = SEEKING;

void setup()
{
  Serial.begin(115200);
  BottleSeeker_setup();
  Grippers_setup();
  Line_setup();
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
