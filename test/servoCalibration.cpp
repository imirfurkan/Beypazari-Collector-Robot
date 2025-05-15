#include <Arduino.h>
#include <Servo.h>

// Calibration: just attach each elbow servo and move to 180°

// static const uint8_t ELBOW_PIN[2] = {37, 39};
// static const uint8_t CAP_PUSHER_PIN = 40; // microswitch actuator
static const uint8_t AYBERK_SRV = 32; // cap‐present switch
static Servo aybSrv;

void setup()
{
  // attach and set both elbows to 180°
  //   elbowSrv[0].attach(ELBOW_PIN[0]);
  //   elbowSrv[0].write(180);

  //   elbowSrv[1].attach(ELBOW_PIN[1]);
  //   elbowSrv[1].write(180);

  //   CapSrv.attach(CAP_PUSHER_PIN);
  //   CapSrv.write(85);

  aybSrv.attach(AYBERK_SRV);
  aybSrv.write(90);
}

void loop()
{
  // nothing needed here
}
