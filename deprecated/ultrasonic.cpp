#include <Arduino.h>
#include "ultrasonic.h"

// Internal physical pin mappings (not exposed)
static const int TRIG_LEFT = 31, ECHO_LEFT = 32;
static const int TRIG_MID = 33, ECHO_MID = 34;
static const int TRIG_RIGHT = 35, ECHO_RIGHT = 36;

// Generic distance helper
unsigned int getDistanceCM(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration =
      pulseIn(echoPin, HIGH, 30000UL);   // timeout at around 30000 microseconds = 30ms ~ 5m
  return duration ? duration / 58 : 999; // division by 58 is equivalent to duration * 0.034 / 2.
}

void initUltrasonics()
{
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_MID, OUTPUT);
  pinMode(ECHO_MID, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
}

// Read distances with logical names
unsigned int getLeftDistance()
{
  return getDistanceCM(TRIG_LEFT, ECHO_LEFT);
}
unsigned int getMiddleDistance()
{
  return getDistanceCM(TRIG_MID, ECHO_MID);
}
unsigned int getRightDistance()
{
  return getDistanceCM(TRIG_RIGHT, ECHO_RIGHT);
}
