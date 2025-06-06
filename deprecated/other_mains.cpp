/////////////////////////////////////////////////////////////////
///// Servo & TOF Main.cpp //////
/////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include "servoTof.h"
#include "motor.h"
#include "motionControl.h"

// --- USER-CONFIGURABLE PARAMETERS ---
const int SERVO_PIN = 13;                     // Digital pin connected to the servo
const int STEP_ANGLE = 5;                     // Servo movement step in degrees
const uint16_t DETECTION_THRESHOLD_MM = 30;   // Distance threshold (mm) for object detection
const unsigned long PAUSE_DURATION_MS = 3000; // How long to pause when an object is detected
const unsigned long MOVE_INTERVAL_MS = 50;    // Interval between each servo step (ms)

// --- TIMING STATE ---
unsigned long lastMoveTime = 0; // Tracks last time the servo was moved

void setup()
{
  // Initialize serial for debugging
  Serial.begin(115200);
  while (!Serial)
  {
  } // Wait for Serial to become available

  // Initialize hardware
  initServo(SERVO_PIN); // Attach servo to specified pin
  initTof();            // Initialize Time-of-Flight (ToF) sensor
  initMotors();
}

void loop()
{
  unsigned long now = millis(); // Get the current system time

  // === SWEEPING STATE ===
  // Only proceed if not currently paused and enough time has passed since last move
  if (!isPaused() && now - lastMoveTime >= MOVE_INTERVAL_MS)
  {
    lastMoveTime = now;

    // Always move forward when sweeping
    setMotionMode(MOVE_FORWARD);
    updateMotors();

    // Update servo angle and write it to hardware
    updateServoDirection(STEP_ANGLE);
    writeServoAngle();

    // Read distance from TOF sensor
    uint16_t dist = readDistance();

    // Print angle and distance for logging
    Serial.print(getCurrentAngle());
    Serial.print("\t");
    Serial.println(dist);

    // If an object is detected within threshold, enter pause state
    if (objectDetected(dist, DETECTION_THRESHOLD_MM))
    {
      Serial.print("Object detected at angle: ");
      Serial.println(getCurrentAngle());
      enterPause(); // Enter PAUSE mode and save the time
    }
  }

  // === PAUSE STATE ===
  // If we are in pause state and the pause duration has elapsed, resume sweep
  if (isPaused() && pauseDone(PAUSE_DURATION_MS))
  {
    Serial.println("Resuming sweep...");
    resetPause();       // Exit pause mode
    lastMoveTime = now; // Prevent immediate servo move after resume
  }
}

/////////////////////////////////////////////////////////////////
///// Only Motor Main //////
/////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "motor.h"
#include "motionControl.h"

void setup()
{
  initMotors();
}

void loop()
{
  setMotionMode(MOVE_FORWARD);
  updateMotors();
  delay(5000);

  setMotionMode(MOVE_BACKWARD);
  updateMotors();
  delay(5000);

  setMotionMode(MOVE_LEFT);
  updateMotors();
  delay(5000);

  setMotionMode(MOVE_RIGHT);
  updateMotors();
  delay(5000);

  setMotionMode(ROTATE_LEFT);
  updateMotors();
  delay(5000);

  setMotionMode(ROTATE_RIGHT);
  updateMotors();
  delay(5000);

  setMotionMode(STOP);
  updateMotors();
  delay(5000);
}