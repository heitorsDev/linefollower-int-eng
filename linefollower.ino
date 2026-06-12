/**
 * linefollower.ino  -- ESP32 PID Line Follower
 * ----------------------------------------------------------------------------
 * Pipeline, once per control tick:
 *
 *    Sensors.read()  ->  error [-1,1]
 *    PID.compute(error)  ->  turn [-1,1]
 *    Chassis.drive(BASE_SPEED, turn)  ->  motor PWM
 *
 * Three classes, one responsibility each (see their headers):
 *    Sensors        -- 5 IR reflectance sensors -> line position error
 *    PIDController  -- error -> steering correction
 *    Chassis        -- (forward, turn) -> L298N motor power
 *
 * All pins / gains / tunables live in Config.h. Nothing to edit here.
 *
 * Sensors use native analogRead() with a raw weighted average -- no
 * thresholds, no calibration phase. It drives immediately on boot.
 *
 * Uses ONLY standard Arduino Language Reference functions (analogRead,
 * analogWrite, digitalWrite, pinMode, millis, Serial) -- no vendor APIs. Port
 * to any analogWrite-capable board by editing Config.h.
 * ----------------------------------------------------------------------------
 */

#include "Config.h"
#include "Sensors.h"
#include "Chassis.h"
#include "PIDController.h"

Sensors       sensors;
Chassis       chassis;
PIDController pid(PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIMIT);

static uint32_t lastLoopMs = 0;

void setup() {
  if (DEBUG_SERIAL) {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    Serial.println(F("\nESP32 PID Line Follower"));
  }

  sensors.begin();
  chassis.begin();
  pid.reset();

  lastLoopMs = millis();
}

void loop() {
  // Fixed-cadence control loop so dt stays consistent.
  uint32_t now = millis();
  if (now - lastLoopMs < LOOP_PERIOD_MS) return;
  float dt = (now - lastLoopMs) / 1000.0f;
  lastLoopMs = now;

  // 1) Where is the line?
  float error = sensors.read();

  // 2) How hard to steer?
  float turn = pid.compute(error, dt);

  // 3) Drive: cruise forward, steer by turn.
  chassis.drive(BASE_SPEED, turn);

  if (DEBUG_SERIAL) {
    static uint32_t lastPrint = 0;
    if (now - lastPrint >= 50) {     // ~20 Hz telemetry, don't flood serial
      lastPrint = now;
      Serial.print(F("err="));   Serial.print(error, 3);
      Serial.print(F("\tturn=")); Serial.print(turn, 3);
      Serial.print(F("\tL="));    Serial.print(chassis.leftPower(), 2);
      Serial.print(F("\tR="));    Serial.print(chassis.rightPower(), 2);
      Serial.print(F("\tline=")); Serial.print(sensors.lineDetected());
      Serial.print(F("\traw="));
      for (uint8_t i = 0; i < 5; i++) { Serial.print(sensors.rawValue(i)); Serial.print(' '); }
      Serial.println();
    }
  }
}
