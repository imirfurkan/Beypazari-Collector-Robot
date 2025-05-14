#include <Arduino.h>
#include <Servo.h>

// from grippers.cpp
const uint8_t CAP_PUSHER_PIN = 39;
const uint8_t CAP_UP_ANGLE = 70;   // TODO
const uint8_t CAP_DOWN_ANGLE = 30; // TODO

// your switch pin
const uint8_t SWITCH_PIN = 7;

Servo capSrv;

void setup()
{
  Serial.begin(115200);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  capSrv.attach(CAP_PUSHER_PIN);
}

void loop()
{
  Serial.println(F("TEST_CAP → pressing & testing cap")); // ← tweaked message

  // 1) Drop the cap-pusher
  capSrv.attach(CAP_PUSHER_PIN);
  capSrv.write(CAP_DOWN_ANGLE);
  delay(700);

  // 2) POLL the microswitch
  if (digitalRead(SWITCH_PIN) == LOW)
  {
    Serial.println(F("→ Cap detected")); // switch closed
  }
  else
  {
    Serial.println(F("→ No cap detected")); // switch open
  }

  // 3) Retract the pusher
  capSrv.write(CAP_UP_ANGLE);
  delay(250);
  capSrv.detach();

  // 4) Pause before repeating
  delay(2000);
}
