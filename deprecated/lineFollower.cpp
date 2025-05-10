#include <Arduino.h>
#include "lineFollower.h"

const int linePins[5] = {A0, A1, A2, A3, A4};

void initLineSensors()
{
  for (int i = 0; i < 5; i++)
  {
    pinMode(linePins[i], INPUT);
  }
}

void readLineSensors(float out[5])
{
  for (int i = 0; i < 5; i++)
  {
    out[i] = analogRead(linePins[i]) / 1023.0;
  }
}

bool allSensorsBlack(const float values[5], float threshold)
{
  for (int i = 0; i < 5; i++)
  {
    if (values[i] >= threshold)
      return false;
  }
  return true;
}
