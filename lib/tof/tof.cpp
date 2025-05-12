#include "tof.h"
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>

static Adafruit_VL53L0X lox;
const byte VL53LOX_InterruptPin = 2;
const byte VL53LOX_ShutdownPin = 9;
volatile bool triggeredObstacle = false;

void VL53LOXISR()
{
  triggeredObstacle = true;
}

void enableTOFInterrupt()
{
  attachInterrupt(digitalPinToInterrupt(VL53LOX_InterruptPin), VL53LOXISR, FALLING);
}

void disableTOFInterrupt()
{
  detachInterrupt(digitalPinToInterrupt(VL53LOX_InterruptPin));
}

void setTOFThresholds(uint16_t low_mm, uint16_t high_mm)
{
  FixPoint1616_t low = (FixPoint1616_t)(low_mm * 65536.0);
  FixPoint1616_t high = (FixPoint1616_t)(high_mm * 65536.0);
  lox.setInterruptThresholds(low, high, false);
}

void setupTOF()
{
  pinMode(VL53LOX_ShutdownPin, OUTPUT);
  digitalWrite(VL53LOX_ShutdownPin, LOW);
  delay(100);
  digitalWrite(VL53LOX_ShutdownPin, HIGH);
  delay(100);

  if (!lox.begin())
  {
    Serial.println("Failed to initialize VL53L0X");
    while (1)
      ;
  }
  pinMode(VL53LOX_InterruptPin, INPUT_PULLUP);

  lox.setGpioConfig(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                    VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW, VL53L0X_INTERRUPTPOLARITY_LOW);

  setTOFThresholds(800, UINT16_MAX);
  lox.setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, false);
  lox.startMeasurement();

  enableTOFInterrupt();
}
