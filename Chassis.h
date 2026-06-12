#pragma once
/**
 * Chassis.h
 * ----------------------------------------------------------------------------
 * Differential-drive chassis: 2 DC motors via an L298N H-bridge.
 *
 * Exposes ONE high-level command:
 *
 *      drive(forward, turn)    forward in [-1,1], turn in [-1,1]
 *
 * It mixes those into per-wheel power (the classic arcade/differential mix):
 *
 *      left  = forward + turn
 *      right = forward - turn
 *
 * then clamps to [-1,1] and renders each wheel as direction (IN1/IN2) plus a
 * PWM duty on the enable pin (ENx) via the ESP32 LEDC peripheral.
 *
 *      turn > 0  -> left speeds up / right slows -> robot yaws RIGHT
 *      turn < 0  -> robot yaws LEFT
 *
 * This matches the Sensors error sign (positive error = line on the right =>
 * positive turn => steer right). So the PID output feeds straight into turn.
 * ----------------------------------------------------------------------------
 */

#include <Arduino.h>
#include "Config.h"

class Chassis {
public:
  Chassis();

  // Configure direction pins + attach LEDC PWM to the enable pins. setup().
  void begin();

  // High-level motion command. Both args in [-1, 1].
  //   forward: + = ahead, - = reverse
  //   turn:    + = steer right, - = steer left
  void drive(float forward, float turn);

  // Cut all power immediately (coast). Use on line-lost-stop or shutdown.
  void stop();

  // ---- accessors (valid after drive()) -----------------------------------
  float leftPower()  const { return _leftPower; }   // [-1, 1]
  float rightPower() const { return _rightPower; }  // [-1, 1]

private:
  float _leftPower;
  float _rightPower;

  // Apply a signed power [-1,1] to one motor: set direction pins + PWM duty.
  void setMotor(const MotorPins& m, bool invert, float power);
};
