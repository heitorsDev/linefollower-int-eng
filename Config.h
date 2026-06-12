#pragma once
/**
 * Config.h
 * ----------------------------------------------------------------------------
 * Central, editable configuration for the line follower.
 *
 * EVERYTHING you might want to change lives here:
 *   - pin assignments (sensors + motor driver)
 *   - sensor geometry / polarity
 *   - PWM characteristics for the L298N
 *   - PID gains and base speed
 *
 * Target board: ESP32 (Arduino IDE / arduino-esp32 core).
 *
 * IMPORTANT ESP32 NOTES
 *   - ADC is 12-bit  -> analogRead() returns 0..4095.
 *   - Use ADC1 pins (GPIO 32-39) for the sensors. ADC2 pins share the radio
 *     and stop working once Wi-Fi/BT is enabled.
 *   - GPIO 34/35/36/39 are INPUT ONLY (no pull-ups, no output) -> perfect for
 *     sensors, useless for motor pins.
 *   - Avoid strapping pins for outputs (0, 2, 12, 15). GPIO12 high at boot can
 *     brick the boot. The motor pins below steer clear of those.
 * ----------------------------------------------------------------------------
 */

#include <Arduino.h>

// ============================================================================
// SENSOR PINOUT  (5x analog IR reflectance sensors, left -> right)
// ============================================================================
// Order MUST be physical left-to-right as the robot drives forward.
// Index:        0      1      2       3       4
// Position: far-L    L    center     R     far-R
// All on ADC1, all input-capable analog pins.
static const uint8_t SENSOR_PINS[5] = { 36, 39, 34, 35, 32 };

// Per-sensor lateral weight used to compute the line position.
// Symmetric around the center sensor. Sign sets which way is "positive error".
// With this layout a positive error  => line is to the ROBOT'S RIGHT.
static const float SENSOR_WEIGHTS[5] = { -2.0f, -1.0f, 0.0f, +1.0f, +2.0f };

// Line polarity. Decides how a raw reading maps to "over the line".
//   true  -> dark line on light floor (line = LOW reflectance = LOW raw). Common.
//   false -> light line on dark floor (line = HIGH raw).
static const bool LINE_IS_DARK = true;

// ============================================================================
// MOTOR / L298N PINOUT
// ============================================================================
// L298N has two channels. Each channel = 2 direction pins (INx) + 1 enable
// pin (ENx) that takes the PWM speed signal.
//
//   LEFT  motor  -> OUT1/OUT2  driven by IN1, IN2, enabled by ENA
//   RIGHT motor  -> OUT3/OUT4  driven by IN3, IN4, enabled by ENB
//
// Remove the ENA/ENB jumpers on the L298N board so the ESP32 PWM controls them.
struct MotorPins {
  uint8_t in1;   // direction A
  uint8_t in2;   // direction B
  uint8_t en;    // enable / PWM (speed)
};

static const MotorPins LEFT_MOTOR  = { /*in1*/ 26, /*in2*/ 27, /*en*/ 25 };
static const MotorPins RIGHT_MOTOR = { /*in1*/ 14, /*in2*/ 13, /*en*/ 33 };

// If a motor spins the wrong way, flip its flag instead of re-wiring.
static const bool LEFT_MOTOR_INVERT  = false;
static const bool RIGHT_MOTOR_INVERT = false;

// ============================================================================
// PWM (LEDC) SETTINGS
// ============================================================================
// L298N is a slow BJT H-bridge: keep PWM frequency modest (~1 kHz) to limit
// switching losses and audible whine. 8-bit resolution -> duty 0..255.
static const uint32_t PWM_FREQ_HZ    = 1000;
static const uint8_t  PWM_RESOLUTION = 8;          // bits
static const uint16_t PWM_MAX_DUTY   = (1 << PWM_RESOLUTION) - 1;  // 255

// Floor below which the motor stalls instead of turning (overcomes static
// friction / driver dropout). Expressed as a duty value, 0..PWM_MAX_DUTY.
// Applied only to non-zero commands.
static const uint16_t PWM_MIN_DUTY = 40;

// ============================================================================
// CONTROL LOOP
// ============================================================================
// Base forward speed, 0..1. The PID only steers; this sets the cruise pace.
static const float BASE_SPEED = 0.45f;

// PID gains. Tune in this order: Kp, then Kd, then Ki (usually small/zero).
// error is in [-1, 1] (see Sensors), output (turn) clamped to [-1, 1].
static const float PID_KP = 0.80f;
static const float PID_KI = 0.00f;
static const float PID_KD = 0.18f;

// Integral clamp (anti-windup), in error*seconds units.
static const float PID_INTEGRAL_LIMIT = 1.0f;

// Control loop period. Keep the loop running at a fixed cadence so dt is sane.
static const uint32_t LOOP_PERIOD_MS = 5;   // 200 Hz

// Print sensor/PID telemetry over Serial for tuning.
static const bool DEBUG_SERIAL = true;
static const uint32_t SERIAL_BAUD = 115200;
