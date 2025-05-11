#pragma once

#include <Arduino.h>

// ── Hardware constants ─────────────────────────────────────
// TB6612 standby pin
static constexpr int MOTOR_STBY_PIN = 30;

// Motor driver pins: { back-left, front-left, front-right, back-right }
static constexpr uint8_t MOTOR_PWM_PIN[4] = {2, 3, 4, 5};
static constexpr uint8_t MOTOR_IN1_PIN[4] = {22, 24, 26, 28};
static constexpr uint8_t MOTOR_IN2_PIN[4] = {23, 25, 27, 29};

// ── Calibration & defaults ────────────────────────────────
// Per-motor “unloaded” scaling factors
static constexpr float MOTOR_K_UNLOADED[4] = {0.974f, 0.808f, 0.835f, 0.750f};
// Logical “forward” direction for rotate functions
static constexpr bool MOTOR_FORWARD_DIR = true;
// Default base speed (0–255)
static constexpr int MOTOR_BASE_SPEED_DEFAULT = 100; // PWM 0-255

static uint8_t lastBaseSpeed = 0xFF; // impossible initial value

inline void ensureMotorSpeed(uint8_t speed)
{
  if (speed != lastBaseSpeed)
  {
    Motor_setBaseSpeed(speed);
    lastBaseSpeed = speed;
  }
}

// ── Public API ────────────────────────────────────────────
/**
 * @brief Configure motor pins & enable TB6612.
 * Must be called once in setup().
 */
void Motor_setup();

/**
 * @brief Change the base speed used for all drive commands.
 */
void Motor_setBaseSpeed(int speed);

/**
 * @brief Immediately stop all four motors.
 */
void Motor_stopAll();

/**
 * @brief Drive straight ahead at the current base speed.
 */
void Motor_driveForward();

/**
 * @brief Rotate in place clockwise (right spin).
 */
void Motor_rotateCW();

/**
 * @brief Rotate in place counter-clockwise (left spin).
 */
void Motor_rotateCCW();