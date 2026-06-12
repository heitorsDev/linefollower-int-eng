#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float integralLimit)
: _kp(kp), _ki(ki), _kd(kd), _integralLimit(integralLimit) {
  reset();
}

void PIDController::reset() {
  _integral   = 0.0f;
  _prevError  = 0.0f;
  _prevMs     = 0;
  _hasPrev    = false;
  _lastOutput = 0.0f;
}

float PIDController::compute(float error, float dtSeconds) {
  // Resolve dt.
  uint32_t now = millis();
  float dt;
  if (dtSeconds > 0.0f) {
    dt = dtSeconds;
  } else if (_hasPrev) {
    dt = (now - _prevMs) / 1000.0f;
  } else {
    dt = 0.0f;   // first call: no derivative/integral contribution
  }
  _prevMs = now;

  // Proportional.
  float p = _kp * error;

  // Integral with anti-windup clamp.
  if (dt > 0.0f) {
    _integral += error * dt;
    _integral = constrain(_integral, -_integralLimit, _integralLimit);
  }
  float i = _ki * _integral;

  // Derivative (skip on first sample / dt == 0 to avoid a spike).
  float d = 0.0f;
  if (_hasPrev && dt > 0.0f) {
    d = _kd * (error - _prevError) / dt;
  }
  _prevError = error;
  _hasPrev = true;

  // Sum + clamp to the chassis turn range.
  float output = p + i + d;
  output = constrain(output, -1.0f, 1.0f);
  _lastOutput = output;
  return output;
}
