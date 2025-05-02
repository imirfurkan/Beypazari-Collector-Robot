#include "servoTof.h"
#include <Adafruit_VL53L0X.h>
#include <Servo.h>

static Servo myServo;
static Adafruit_VL53L0X tof;
static int currentAngle = 0;
static int dir = 1;
static unsigned long pauseStart = 0;
static bool paused = false;

void initServo(int servoPin)
{
  myServo.attach(servoPin);
}

void initTof()
{
  if (!tof.begin())
  {
    Serial.println("VL53L0X not found!");
    while (1)
    {
    }
  }
  tof.setMeasurementTimingBudgetMicroSeconds(50000); // TODO: why 50000?
  if (!tof.startRangeContinuous(33))
  {
    Serial.println("Failed to start TOF continuous mode!");
    while (1)
    {
    }
  }
}

void updateServoDirection(int stepAngle)
{
  if (paused)
    return;
  currentAngle += dir * stepAngle;
  if (currentAngle > 180)
  {
    currentAngle = 180;
    dir = -1;
  }
  else if (currentAngle < 0)
  {
    currentAngle = 0;
    dir = 1;
  }
}

void writeServoAngle()
{
  if (!paused)
  {
    myServo.write(currentAngle);
  }
}

uint16_t readDistance()
{
  return tof.readRangeResult();
}

bool objectDetected(uint16_t dist, uint16_t threshold)
{
  return !tof.timeoutOccurred() && dist > 0 && dist < threshold;
}

void enterPause()
{
  paused = true;
  pauseStart = millis();
}

bool isPaused()
{
  return paused;
}

bool pauseDone(unsigned long duration)
{
  return millis() - pauseStart >= duration;
}

void resetPause()
{
  paused = false;
}

int getCurrentAngle()
{
  return currentAngle;
}

void resetServo()
{
  currentAngle = 90;
  dir = 1; // add extra logic to here later.
  myServo.write(currentAngle);
}