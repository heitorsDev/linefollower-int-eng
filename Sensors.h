#pragma once
/**
 * Sensors.h
 * ----------------------------------------------------------------------------
 * 5-channel IR reflectance sensor array.
 *
 * Job: turn 5 raw analogRead() values into ONE number -- the line's lateral
 * position under the robot -- expressed as an "angular" error in [-1, +1].
 *
 *      error < 0  : line is to the LEFT   -> robot must steer left
 *      error = 0  : line centered
 *      error > 0  : line is to the RIGHT  -> robot must steer right
 *
 * METHOD (weighted average / "center of mass" of the line). No thresholds,
 * no calibration -- straight from the raw ADC values:
 *
 *      position = sum( weight[i] * v[i] ) / sum( v[i] )
 *
 *   v[i] is the raw reading mapped so it is LARGE when sensor i is over the
 *   line (polarity handled by LINE_IS_DARK). Dividing by the max |weight|
 *   normalizes position into [-1, 1]. Continuous, sub-sensor resolution.
 * ----------------------------------------------------------------------------
 */

#include <Arduino.h>
#include "Config.h"

class Sensors {
public:
  Sensors();

  // Configure ADC + pins. Call once from setup().
  void begin();

  // Read all sensors and compute the line error in [-1, 1].
  float read();

  // ---- accessors (valid after read()) ------------------------------------
  float lastError() const { return _error; }       // [-1, 1]
  bool  lineDetected() const { return _lineSeen; }  // any signal under the bar?
  int   rawValue(uint8_t i) const { return _raw[i]; }

private:
  int   _raw[5];        // last raw analogRead values, 0..4095
  float _error;         // last computed error, [-1, 1]
  bool  _lineSeen;      // was there meaningful signal on the last read?
  float _lastError;     // remembered for the "line lost" fallback
  float _maxWeight;     // max |weight|, normalizes position to [-1, 1]
};
