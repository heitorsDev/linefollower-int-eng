#include "Chassis.h"

// ----------------------------------------------------------------------------
// LEDC PWM API shim.
// arduino-esp32 core 3.x changed the LEDC API:
//   - 3.x  : ledcAttach(pin, freq, resolution); ledcWrite(pin, duty);
//   - <3.x : ledcSetup(channel, freq, resolution); ledcAttachPin(pin, channel);
//            ledcWrite(channel, duty);
// We hide the difference so the rest of the class just calls pwmAttach/pwmWrite
// with the PIN. On the old core we allocate one channel per enable pin.
// ----------------------------------------------------------------------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  static inline void pwmAttach(uint8_t pin) {
    ledcAttach(pin, PWM_FREQ_HZ, PWM_RESOLUTION);
  }
  static inline void pwmWrite(uint8_t pin, uint32_t duty) {
    ledcWrite(pin, duty);
  }
#else
  static uint8_t _nextChannel = 0;
  // Map enable pin -> channel. Two motors -> at most 2 channels here.
  static int8_t _chanForPin[40];
  static bool   _chanInit = false;
  static inline void pwmAttach(uint8_t pin) {
    if (!_chanInit) { for (int i = 0; i < 40; i++) _chanForPin[i] = -1; _chanInit = true; }
    uint8_t ch = _nextChannel++;
    _chanForPin[pin] = ch;
    ledcSetup(ch, PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcAttachPin(pin, ch);
  }
  static inline void pwmWrite(uint8_t pin, uint32_t duty) {
    int8_t ch = _chanForPin[pin];
    if (ch >= 0) ledcWrite(ch, duty);
  }
#endif

Chassis::Chassis() : _leftPower(0.0f), _rightPower(0.0f) {}

void Chassis::begin() {
  // Direction pins as outputs.
  pinMode(LEFT_MOTOR.in1, OUTPUT);
  pinMode(LEFT_MOTOR.in2, OUTPUT);
  pinMode(RIGHT_MOTOR.in1, OUTPUT);
  pinMode(RIGHT_MOTOR.in2, OUTPUT);

  // Enable pins driven by PWM.
  pwmAttach(LEFT_MOTOR.en);
  pwmAttach(RIGHT_MOTOR.en);

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
  pwmWrite(m.en, duty);
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
  pwmWrite(LEFT_MOTOR.en, 0);
  pwmWrite(RIGHT_MOTOR.en, 0);
}
