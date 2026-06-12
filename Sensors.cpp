#include "Sensors.h"

Sensors::Sensors()
: _error(0.0f), _lineSeen(false), _lastError(0.0f) {
  for (uint8_t i = 0; i < 5; i++) _raw[i] = 0;

  // Precompute the largest absolute weight for normalization.
  _maxWeight = 0.0f;
  for (uint8_t i = 0; i < 5; i++) {
    float w = fabsf(SENSOR_WEIGHTS[i]);
    if (w > _maxWeight) _maxWeight = w;
  }
  if (_maxWeight <= 0.0f) _maxWeight = 1.0f;  // guard against all-zero weights
}

void Sensors::begin() {
  // Standard Arduino analog input. analogReadResolution() sets the bit depth
  // so analogRead() returns 0..ADC_MAX (ESP32 = 12 bits -> 0..4095).
  analogReadResolution(ADC_RESOLUTION_BITS);
  for (uint8_t i = 0; i < 5; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }
}

float Sensors::read() {
  float weightedSum = 0.0f;   // sum( w[i] * v[i] )
  float total       = 0.0f;   // sum( v[i] )

  for (uint8_t i = 0; i < 5; i++) {
    _raw[i] = analogRead(SENSOR_PINS[i]);   // native Arduino read, 0..ADC_MAX

    // Map raw so the value is LARGE when this sensor sits over the line.
    // Dark line  -> low reflectance -> low raw -> invert.
    // Light line -> high reflectance -> use raw directly.
    float v = LINE_IS_DARK ? ((float)ADC_MAX - (float)_raw[i]) : (float)_raw[i];

    weightedSum += SENSOR_WEIGHTS[i] * v;
    total       += v;
  }

  // total is tiny only when every sensor is fully off the line.
  _lineSeen = total > 1.0f;

  if (_lineSeen) {
    float position = weightedSum / total;   // in [-maxW, +maxW]
    _error = position / _maxWeight;          // in [-1, 1]
    _error = constrain(_error, -1.0f, 1.0f);
    _lastError = _error;
  } else {
    // Line lost: keep steering hard toward the side we last saw it.
    _error = (_lastError >= 0.0f) ? 1.0f : -1.0f;
  }

  return _error;
}
