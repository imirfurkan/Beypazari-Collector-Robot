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

// New: clear the VL53L0X’s internal interrupt so the pin goes HIGH again
void clearTOFInterrupt()
{
  lox.clearInterruptMask();
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
    while (true)
      ;
  }
  pinMode(VL53LOX_InterruptPin, INPUT_PULLUP);

  lox.setGpioConfig(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                    VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW, VL53L0X_INTERRUPTPOLARITY_LOW);

  setTOFThresholds(800, UINT16_MAX);

  lox.setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, false);
  lox.startMeasurement();

  // lox.setMeasurementTimingBudgetMicroSeconds(20000); // 20 ms integration

  enableTOFInterrupt();
}

/// TOF test for main.cpp

// #include <Arduino.h>
// #include "tof.h"

// // This flag is set to true in your VL53LOXISR() (tof.cpp)
// extern volatile bool triggeredObstacle;

// void setup()
// {
//   Serial.begin(115200);
//   while (!Serial)
//   {
//     delay(10);
//   }
//   Serial.println(F("=== TOF Interrupt Test Started ==="));

//   // Initialize the VL53L0X and arm its interrupt
//   setupTOF();
// }

// void loop()
// {
//   // If our ISR pulled the flag low→high, report it
//   if (triggeredObstacle)
//   {
//     Serial.println(F("🚨 TOF interrupt triggered!"));
//     clearTOFInterrupt(); // clear the sensor’s own interrupt
//     triggeredObstacle = false;
//   }

//   // small pause to avoid flooding the serial port
//   delay(10);
// }
