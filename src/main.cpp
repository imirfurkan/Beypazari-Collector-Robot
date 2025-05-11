#include <Arduino.h>
#include "Grippers.h"

void setup()
{
  // start serial for debug (optional)
  Serial.begin(9600);
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
