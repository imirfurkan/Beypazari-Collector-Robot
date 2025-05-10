#include <Arduino.h>
#include <NewPing.h>

// ────────────────────────────────────────────────
//  Pin map (Mega)
// ────────────────────────────────────────────────
const int STBY = 30; // TB6612 standby

// Motors
const int PWM1 = 2;
const int IN1_1 = 22;
const int IN2_1 = 23; // back-left
const int PWM2 = 3;
const int IN1_2 = 24;
const int IN2_2 = 25; // front-left
const int PWM3 = 4;
const int IN1_3 = 26;
const int IN2_3 = 27; // front-right
const int PWM4 = 5;
const int IN1_4 = 28;
const int IN2_4 = 29; // back-right

// HC-SR04 Ultrasonic Sensor
const int TRIG_RIGHT = 31;
const int ECHO_RIGHT = 32;
const int TRIG_LEFT = 35;
const int ECHO_LEFT = 36;
const int TRIG_MIDDLE = 33;
const int ECHO_MIDDLE = 34;

// ────────────────────────────────────────────────
//  Behaviour parameters
// ────────────────────────────────────────────────
const unsigned int DIST_THRESHOLD = 50; // cm
int baseSpeed = 100;                    // PWM 0-255
const int BASE_SPEED_DEFAULT = 100;
const int BASE_SPEED_TURNING = 40;
float K_unloaded[4] = {0.974, 0.808, 0.835, 0.750};
const bool forwardDir = true;
// const int ROTATE_STEP_MS = 40;             // ← rotate chunk
const unsigned int GRIPPER_THRESHOLD = 20; // cm – “too close” distance

const unsigned long ROTATE_EXTRA_MS = 2000; // keep spinning this long (ms)
bool rotating = false;                      // are we inside that window?
int rotDir = 0;                             // ‑1 = CCW , +1 = CW
unsigned long rotEnd = 0;                   // millis() deadline
const uint8_t FALLBACK_DEBOUNCE = 4;        // require 5 consecutive “no‐sonar” passes
uint8_t fallbackCount = 0;
enum Action
{
  NONE,
  ROTATING,
  MOVING
};
Action prevAct = NONE;

NewPing sonarL(TRIG_LEFT, ECHO_LEFT, DIST_THRESHOLD);
NewPing sonarM(TRIG_MIDDLE, ECHO_MIDDLE, DIST_THRESHOLD);
NewPing sonarR(TRIG_RIGHT, ECHO_RIGHT, DIST_THRESHOLD);

// static bool centerSeen = false;            // <‑‑ NEW  (remember once)
// ────────────────────────────────────────────────
//  Helpers
// ────────────────────────────────────────────────
int trimmedPWM(int i)
{
  return constrain(int(baseSpeed * K_unloaded[i]), 0, 255);
}
void driveMotor(int idx, int pwm, int in1, int in2, bool fw)
{
  digitalWrite(in1, fw ? HIGH : LOW);
  digitalWrite(in2, fw ? LOW : HIGH);
  analogWrite(pwm, trimmedPWM(idx));
}
void stopAll()
{
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  analogWrite(PWM3, 0);
  analogWrite(PWM4, 0);
}
void driveForward()
{
  driveMotor(0, PWM1, IN1_1, IN2_1, true);
  driveMotor(1, PWM2, IN1_2, IN2_2, true);
  driveMotor(2, PWM3, IN1_3, IN2_3, true);
  driveMotor(3, PWM4, IN1_4, IN2_4, true);
}
void rotateCCW()
{
  driveMotor(0, PWM1, IN1_1, IN2_1, !forwardDir);
  driveMotor(1, PWM2, IN1_2, IN2_2, !forwardDir);
  driveMotor(2, PWM3, IN1_3, IN2_3, forwardDir);
  driveMotor(3, PWM4, IN1_4, IN2_4, forwardDir);
}
void rotateCW()
{
  driveMotor(0, PWM1, IN1_1, IN2_1, forwardDir);
  driveMotor(1, PWM2, IN1_2, IN2_2, forwardDir);
  driveMotor(2, PWM3, IN1_3, IN2_3, !forwardDir);
  driveMotor(3, PWM4, IN1_4, IN2_4, !forwardDir);
}
//  Read one HC-SR04 (blocking)
unsigned int getDistance(int trig, int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 30000UL);
  return us ? us / 58 : 999; // cm or “far”
}

// ────────────────────────────────────────────────
//  SETUP
// ────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // motor pins
  int pins[] = {PWM1, IN1_1, IN2_1, PWM2, IN1_2, IN2_2, PWM3, IN1_3, IN2_3, PWM4, IN1_4, IN2_4};
  for (int p : pins)
    pinMode(p, OUTPUT);

  // sonar pins
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_MIDDLE, OUTPUT);
  pinMode(ECHO_MIDDLE, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  Serial.println(F("Bottle-seeker v2 – continuous spin & sensor map fixed"));
}

// State machine
enum RobotState
{
  SEARCH,        // Move forward and check the sensors, when obj_detected APPROACH mode
  APPROACH,      // rotate car and move, when close enough for gripper transition to HANDLE_OBJECT
  HANDLE_OBJECT, // Close gripper, inspect bottle, if full -> open gripper, turn 90 deg., SEARCH
                 // mode, if empty -> is it 4th bottle?, if yes LINESEARCH mode, if no -> bring new
                 // gripper, bottle_count++, SEARCH mode
  LINE_SEARCH,   // find line, then LINEFOLLOW mode
  LINE_FOLLOW,   // follow line, when line finish deliver bottles
  // interrrup, if time=4min -> LINESEARCH
};

RobotState currentState = SEARCH;

// ────────────────────────────────────────────────
//  LOOP  (simple state machine)
// ────────────────────────────────────────────────
void loop()
{

  //--------------------------------------------------
  // In loop variables
  //--------------------------------------------------
  // unsigned long now = millis();
  // unsigned int dL = getDistance(TRIG_LEFT, ECHO_LEFT);
  // unsigned int dM = getDistance(TRIG_MIDDLE, ECHO_MIDDLE);
  // unsigned int dR = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  unsigned int rawL = sonarL.ping_cm();
  unsigned int dL = (rawL == 0) ? (DIST_THRESHOLD + 1) : rawL;

  unsigned int rawM = sonarM.ping_cm();
  unsigned int dM = (rawM == 0) ? (DIST_THRESHOLD + 1) : rawM;

  unsigned int rawR = sonarR.ping_cm();
  unsigned int dR = (rawR == 0) ? (DIST_THRESHOLD + 1) : rawR;

  bool objAny = (dL < DIST_THRESHOLD) || (dM < DIST_THRESHOLD) || (dR < DIST_THRESHOLD);
  bool objL = (dL < DIST_THRESHOLD);
  bool objR = (dR < DIST_THRESHOLD);
  bool objM = (dM < DIST_THRESHOLD);
  bool objGrip = dM < GRIPPER_THRESHOLD;

  switch (currentState)
  {

  case SEARCH:
  {

    if (!objAny)
    {
      Serial.println("no object yet");
      driveForward();
      delay(50);
      return;
    }

    currentState = APPROACH; // else if (objAny)
    Serial.println("search done");
    break;
  } // END of Search mode

  case APPROACH:
  {
    // 1) Gripper‐close check … (unchanged)
    if (objGrip)
    {
      Serial.println(F("Object too close! Stopping & activating gripper."));
      Serial.print(dM);
      stopAll();
      // activateGripper();
      delay(5000); // give gripper time to close
      return;      // skip the rest of loop()
    } // END of if

    /* ──── timed rotation window with early-exit ──── */
    if (rotating)
    {
      fallbackCount = 0;
      Serial.println(F("APPROACH: still rotating…"));
      if (rotDir < 0)
        rotateCCW();
      else
        rotateCW();

      bool centreDetected = objM;
      Serial.print(centreDetected);
      if (centreDetected || millis() >= rotEnd)
      {
        delay(500);
        rotating = false;
        baseSpeed = BASE_SPEED_DEFAULT;
        stopAll();
        delay(300); // #TODO
        Serial.println(centreDetected ? F("APPROACH: rotation aligned → surging forward")
                                      : F("APPROACH: rotation timeout → falling back"));

        if (centreDetected)
        {
          driveForward();
          delay(200);
        }
      }
      break; // remain in APPROACH
    }

    // ── normal steering logic ──

    // Only-middle sees → drive straight
    if (!objL && objM && !objR)
    {
      fallbackCount = 0;
      prevAct = MOVING;
      baseSpeed = BASE_SPEED_DEFAULT;
      Serial.println(F("APPROACH: centre only → driveForward"));
      driveForward();
      return;
    }
    // Left-side sees → start CCW spin
    else if ((objL && !objM && !objR) || (objL && objM && !objR))
    {
      fallbackCount = 0;
      baseSpeed = BASE_SPEED_TURNING;
      rotDir = -1;
      rotating = true; // <— THIS makes the top if(rotating) run next loop
      rotEnd = millis() + ROTATE_EXTRA_MS;
      Serial.println(F("APPROACH: start rotating CCW"));
      rotateCCW();
      return;
    }
    // Right-side sees → start CW spin
    else if ((!objL && !objM && objR) || (!objL && objM && objR))
    {
      fallbackCount = 0;
      baseSpeed = BASE_SPEED_TURNING;
      rotDir = +1;
      rotating = true; // <— THIS makes the top if(rotating) run next loop
      rotEnd = millis() + ROTATE_EXTRA_MS;
      Serial.println(F("APPROACH: start rotating CW"));
      rotateCW();
      return;
    }
    // Edge cases → forward
    else if ((objL && !objM && objR) || (objL && objM && objR))
    {
      fallbackCount = 0;
      prevAct = MOVING;
      baseSpeed = BASE_SPEED_DEFAULT;
      Serial.println(F("APPROACH: both-sides → driveForward"));
      driveForward();
      return;
    }
    // Debounced fallback → SEARCH
    else
    {
      fallbackCount++;
      if (fallbackCount >= FALLBACK_DEBOUNCE)
      {
        currentState = SEARCH;
        prevAct = NONE;
        fallbackCount = 0;
        Serial.println(F("APPROACH: fallback → SEARCH (debounced)"));
      }
      else
      {
        Serial.print(F("APPROACH: fallback debounce #"));
        Serial.println(fallbackCount);
      }
      return;
    }
  }
  break; // end of APPROACH
  }
}