#pragma once
/**
 * PIDController.h
 * ----------------------------------------------------------------------------
 * Generic PID controller, tuned here for line-following steering.
 *
 *      output = Kp*e + Ki*∫e dt + Kd*de/dt
 *
 *   - input  : line error from Sensors, in [-1, 1] (setpoint is 0 = centered).
 *   - output : steering "turn" command, clamped to [-1, 1], fed to Chassis.
 *
 * Practical safeguards baked in:
 *   - dt measured from millis() so gains are time-consistent regardless of
 *     loop jitter.
 *   - integral anti-windup (clamped) so a long off-line stretch can't saturate.
 *   - output clamped to [-1, 1] to match the chassis turn range.
 *
 * TUNING (on a real track): raise Kp until it tracks but oscillates, add Kd to
 * damp the oscillation, add a little Ki only if it consistently rides off-
 * center. Start from the values in Config.h.
 * ----------------------------------------------------------------------------
 */

#include <Arduino.h>
#include "Config.h"

class PIDController {
public:
  PIDController(float kp, float ki, float kd, float integralLimit);

  // Reset accumulated state (call when (re)starting the run).
  void reset();

  // Compute steering from the current error. dtSeconds <= 0 makes the method
  // measure dt itself from millis(); pass your loop dt for determinism.
  float compute(float error, float dtSeconds = -1.0f);

  // Live gain tweaks (e.g. from a serial tuner).
  void setGains(float kp, float ki, float kd) { _kp = kp; _ki = ki; _kd = kd; }

  // ---- accessors ----------------------------------------------------------
  float lastOutput()     const { return _lastOutput; }
  float integralTerm()   const { return _integral; }

private:
  float _kp, _ki, _kd;
  float _integralLimit;

  float _integral;       // accumulated ∫e dt
  float _prevError;      // for the derivative
  uint32_t _prevMs;      // for internal dt measurement
  bool  _hasPrev;        // first call guard
  float _lastOutput;
};
