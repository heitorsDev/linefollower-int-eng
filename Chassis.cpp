#include "Chassis.h"

// Speed is produced with the standard Arduino analogWrite() on the L298N
// enable pins (ENA/ENB). Direction comes from digitalWrite() on the IN pins.
// Only documented Arduino Language Reference functions are used here:
//   pinMode, digitalWrite, analogWrite, analogWriteResolution.

Chassis::Chassis() : _leftPower(0.0f), _rightPower(0.0f) {}

void Chassis::begin() {
  // Direction pins as outputs.
  pinMode(LEFT_MOTOR.in1, OUTPUT);
  pinMode(LEFT_MOTOR.in2, OUTPUT);
  pinMode(RIGHT_MOTOR.in1, OUTPUT);
  pinMode(RIGHT_MOTOR.in2, OUTPUT);

  // Enable pins carry the PWM speed signal.
  pinMode(LEFT_MOTOR.en, OUTPUT);
  pinMode(RIGHT_MOTOR.en, OUTPUT);

  // Set analogWrite() duty range to match PWM_MAX_DUTY.
  analogWriteResolution(PWM_RESOLUTION);

  stop();
}

void Chassis::setMotor(const MotorPins& m, bool invert, float power) {
  power = constrain(power, -1.0f, 1.0f);
  if (invert) power = -power;

  // Direction from sign.
  bool forward = power >= 0.0f;
  digitalWrite(m.in1, forward ? HIGH : LOW);
  digitalWrite(m.in2, forward ? LOW  : HIGH);

  // Magnitude -> duty, with a minimum floor so small commands still move.
  float mag = fabsf(power);
  uint32_t duty = 0;
  if (mag > 0.001f) {
    duty = (uint32_t)(mag * PWM_MAX_DUTY);
    if (duty < PWM_MIN_DUTY) duty = PWM_MIN_DUTY;
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
  }
  analogWrite(m.en, duty);
}

void Chassis::drive(float forward, float turn) {
  forward = constrain(forward, -1.0f, 1.0f);
  turn    = constrain(turn,    -1.0f, 1.0f);

  // Differential mix.
  _leftPower  = forward + turn;
  _rightPower = forward - turn;

  // Clamp after mixing (a hard turn at speed can exceed 1).
  _leftPower  = constrain(_leftPower,  -1.0f, 1.0f);
  _rightPower = constrain(_rightPower, -1.0f, 1.0f);

  setMotor(LEFT_MOTOR,  LEFT_MOTOR_INVERT,  _leftPower);
  setMotor(RIGHT_MOTOR, RIGHT_MOTOR_INVERT, _rightPower);
}

void Chassis::stop() {
  _leftPower = _rightPower = 0.0f;
  digitalWrite(LEFT_MOTOR.in1, LOW);
  digitalWrite(LEFT_MOTOR.in2, LOW);
  digitalWrite(RIGHT_MOTOR.in1, LOW);
  digitalWrite(RIGHT_MOTOR.in2, LOW);
  analogWrite(LEFT_MOTOR.en, 0);
  analogWrite(RIGHT_MOTOR.en, 0);
}
