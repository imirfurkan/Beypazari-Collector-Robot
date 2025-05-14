#pragma once

#include <Arduino.h>

// ── Hardware constants ─────────────────────────────────────
// TB6612 standby pin
static constexpr int MOTOR_STBY_PIN = 30;

// Motor driver pins: { back-left, front-left, front-right, back-right }
static constexpr uint8_t MOTOR_PWM_PIN[4] = {3, 4, 5, 6};
static constexpr uint8_t MOTOR_IN1_PIN[4] = {22, 24, 26, 28};
static constexpr uint8_t MOTOR_IN2_PIN[4] = {23, 25, 27, 29};

// ── Calibration & defaults ────────────────────────────────
// Per-motor “unloaded” scaling factors
static constexpr float MOTOR_K_UNLOADED[4] = {0.974f, 0.808f, 0.835f, 0.750f};
static constexpr bool MOTOR_FORWARD_DIR = true;
static constexpr int MOTOR_SPEED_DEFAULT = 150; // PWM 0-255
static constexpr int MOTOR_PIVOT_SPEED = 130;
static constexpr int MOTOR_GENTLE_SPEED = 130;
static constexpr int MOTOR_DOUBLECHECK_SPEED = 110;
static constexpr float MOTOR_STEER_PCT = 0.4f; // 0.0 = hard turn, 1.0 = straight

static uint8_t lastBaseSpeed = 0xFF; // impossible initial value

// ── Public API ────────────────────────────────────────────
void Motor_setup();
void Motor_setBaseSpeed(int speed);
void Motor_stopAll();
void Motor_driveForward();
void Motor_driveBackward();
void Motor_rotateCW();
void Motor_rotateCCW();

/**
 * @brief Gentle arc to the left using pre-set gentle speed & steer percentage.
 */
void Motor_gentleLeft();

/**
 * @brief Gentle arc to the right using pre-set gentle speed & steer percentage.
 */
void Motor_gentleRight();

inline void ensureMotorSpeed(uint8_t speed)
{
  if (speed != lastBaseSpeed)
  {
    Motor_setBaseSpeed(speed);
    lastBaseSpeed = speed;
  }
}
