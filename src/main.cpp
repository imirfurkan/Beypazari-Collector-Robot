#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#include "bottleSeeker.h"
#include "grippers.h"
#include "motors.h"
#include "line.h"

// ── Robot State Machine ─────────────────────────────────
enum RobotState { SEEKING, GRIPPING, NAVIGATING, INTERRUPT };
static RobotState robotState = SEEKING;
static RobotState previousState = SEEKING;

// ── VL53L0X Configuration ───────────────────────────────
const byte VL53LOX_InterruptPin = 2;
const byte VL53LOX_ShutdownPin  = 9;
volatile bool triggeredObstacle = false;
Adafruit_VL53L0X lox;

// ISR: simply set the flag
void VL53LOXISR() {
  triggeredObstacle = true;
}

// Enable / disable the hardware interrupt
void enableTOFInterrupt() {
  attachInterrupt(digitalPinToInterrupt(VL53LOX_InterruptPin),
                  VL53LOXISR, FALLING);
}
void disableTOFInterrupt() {
  detachInterrupt(digitalPinToInterrupt(VL53LOX_InterruptPin));
}

// Set the threshold window: here we only care about "below 80 mm"
void setTOFThresholds(uint16_t low_mm, uint16_t high_mm) {
  FixPoint1616_t low  = (FixPoint1616_t)(low_mm * 65536.0);
  FixPoint1616_t high = (FixPoint1616_t)(high_mm * 65536.0);
  // window=false so it triggers only when crossing thresholds
  lox.setInterruptThresholds(low, high, false);
}

// All VL53L0X boot & config in one place
void setupTOF() {
  // 1) Hardware reset via XSHUT
  pinMode(VL53LOX_ShutdownPin, OUTPUT);
  digitalWrite(VL53LOX_ShutdownPin, LOW);
  delay(100);
  digitalWrite(VL53LOX_ShutdownPin, HIGH);
  delay(100);

  // 2) Wire up and begin
  if (!lox.begin()) {
    Serial.println("❌ Failed to initialize VL53L0X");
    while (1);
  }
  pinMode(VL53LOX_InterruptPin, INPUT_PULLUP);

  // 3) Configure GPIO1 to go LOW when below low-threshold
  lox.setGpioConfig(
    VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
    VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
    VL53L0X_INTERRUPTPOLARITY_LOW
  );

  // 4) Trigger only when range < 80 mm
  setTOFThresholds(80, UINT16_MAX);

  // 5) Start continuous measurements
  lox.setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, false);
  lox.startMeasurement();

  // 6) Attach interrupt on FALLING (only when line goes high→low)
  enableTOFInterrupt();
}

// ── Standard Arduino Setup ──────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  setupTOF();
  BottleSeeker_setup();
  Grippers_setup();
  Line_setup();
}

// ── Main Loop with Obstacle-Interrupt Handling ─────────
void loop() {
  // If TOF ISR fired, mask further TOF interrupts and switch state
  if (triggeredObstacle) {
    disableTOFInterrupt();
    previousState = robotState;
    robotState    = INTERRUPT;
    triggeredObstacle = false;
  }

  switch (robotState) {
    case SEEKING:
      if (BottleSeeker_loop())
        robotState = GRIPPING;
      break;

    case GRIPPING:
      if (Grippers_loop()) {
        if (bottleRejected()) {
          Motor_driveBackward();
          delay(1000);
          Motor_rotateCW();
          delay(1000);
          Motor_stopAll();
        }
        robotState = NAVIGATING;
      }
      break;

    case NAVIGATING:
      if (Line_loop()) {
        Serial.println(F(">> TARGET AREA REACHED! Placing bottles..."));
        Motor_driveForward();
        delay(1000);
        Motor_stopAll();
        robotState = SEEKING;
      }
      break;

    case INTERRUPT:
      Serial.println(F("🚧 Obstacle (<80 mm) detected! Avoiding..."));
      Motor_rotateCW();
      delay(random(500, 701));  // tune this for your avoidance turn
      Motor_stopAll();
      enableTOFInterrupt();             // re-arm TOF interrupt
      robotState = previousState;       // resume prior activity
      break;
  }

  delay(10);
}