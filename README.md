# ESP32 PID Line Follower

A 5-sensor, 2-motor line follower for the **ESP32**, programmed via the **Arduino IDE**.
Clean class separation, configurable pinout, continuous PID steering.

## Architecture

```
  ┌───────────┐  error[-1,1]  ┌────────────────┐  turn[-1,1]  ┌───────────┐
  │  Sensors  │ ────────────▶ │ PIDController  │ ───────────▶ │  Chassis  │
  │ 5× IR ADC │               │  Kp·Ki·Kd      │              │ L298N PWM │
  └───────────┘               └────────────────┘              └───────────┘
```

| Class           | Responsibility                                                       |
|-----------------|----------------------------------------------------------------------|
| `Sensors`       | Native `analogRead` 5 IR → raw weighted average → angular error `[-1, 1]` (no threshold/calibration) |
| `PIDController` | error → steering correction `turn ∈ [-1, 1]` (with anti-windup)      |
| `Chassis`       | `(forward, turn)` → differential mix → direction pins + LEDC PWM      |

Each tick (`loop()`): `error = sensors.read()` → `turn = pid.compute(error)` → `chassis.drive(BASE_SPEED, turn)`.

### Sign convention (kept consistent end-to-end)
- `error > 0` → line is to the **right** → `turn > 0` → robot yaws **right**. Line re-centers.

## Files

| File               | Purpose                                            |
|--------------------|----------------------------------------------------|
| `Config.h`         | **All** pins, gains, PWM, speeds — edit here only  |
| `Sensors.h/.cpp`   | IR array + raw weighted-average line estimator     |
| `Chassis.h/.cpp`   | L298N differential drive (LEDC PWM)                |
| `PIDController.h/.cpp` | PID with dt-aware integral/derivative          |
| `sketch.ino`       | Wiring of the three classes + control loop         |

> Arduino IDE compiles every `.ino`/`.h`/`.cpp` in the sketch folder as tabs — keep them together.

## Wiring

### IR sensors → ESP32 (analog, ADC1 only)
Physical order **left → right** as the robot drives forward.

| Sensor   | far-L | L   | center | R   | far-R |
|----------|-------|-----|--------|-----|-------|
| GPIO     | 36    | 39  | 34     | 35  | 32    |

> ADC1 pins survive Wi-Fi; **ADC2 pins (e.g. GPIO 0/2/4/12-15/25-27) stop reading once the radio is on**. 34/35/36/39 are input-only — ideal here.

### L298N → ESP32

| L298N pin | Function            | GPIO |
|-----------|---------------------|------|
| ENA       | Left speed (PWM)    | 25   |
| IN1       | Left dir A          | 26   |
| IN2       | Left dir B          | 27   |
| IN3       | Right dir A         | 14   |
| IN4       | Right dir B         | 13   |
| ENB       | Right speed (PWM)   | 33   |

- **Remove the ENA/ENB jumpers** so the ESP32 PWM controls speed.
- Common ground: L298N GND ↔ ESP32 GND ↔ battery −.
- Motor supply on L298N `+12V`; keep its onboard 5V regulator **off** the ESP32 (power the ESP32 from USB/its own regulator) unless you know the regulator can handle it.
- L298N drops ~2 V across its BJT bridge — feed it a battery with headroom (e.g. 7.4 V LiPo for 6 V motors).

All of the above is just the **default**. Change any pin in `Config.h` (`SENSOR_PINS`, `LEFT_MOTOR`, `RIGHT_MOTOR`).

## Quick start

1. Arduino IDE → install **esp32 by Espressif** (Boards Manager). Select your ESP32 board.
2. Open the folder; all tabs load. Upload.
3. It drives immediately on boot (no calibration). Open Serial Monitor @ **115200** for telemetry.

## Tuning (in `Config.h`)

1. **Direction first.** If a wheel spins backward, set `LEFT_MOTOR_INVERT` / `RIGHT_MOTOR_INVERT`.
2. **Polarity.** Dark line on light floor → `LINE_IS_DARK = true`. Light line → `false`.
3. **PID** (do in order):
   - `PID_KP`: raise until it follows but wobbles.
   - `PID_KD`: raise to damp the wobble.
   - `PID_KI`: only if it consistently rides off-center; keep tiny.
4. `BASE_SPEED`: raise once steering is stable.
5. `PWM_MIN_DUTY`: lowest duty that still moves the motors (overcome stall).

## ESP32-specific notes baked into the code

- **ADC** is 12-bit (`analogReadResolution(12)`, 0–4095) with 11 dB attenuation for the full ~0–3.3 V swing.
- **PWM** uses the LEDC peripheral, **not** `analogWrite`. `Chassis.cpp` auto-selects the API: `ledcAttach()/ledcWrite(pin,…)` on core 3.x, `ledcSetup()/ledcAttachPin()/ledcWrite(channel,…)` on 2.x.

## References

- [Line Follower Robot (with PID controller) — Arduino Project Hub](https://projecthub.arduino.cc/anova9347/line-follower-robot-with-pid-controller-01813f) — weighted-position error + PID structure.
- [pid-line-follower-esp32-mini (GitHub)](https://github.com/youcefboubidi/pid-line-follower-esp32-mini) — ESP32 + LEDC PWM PID follower.
- [ESP32 with DC Motor and L298N — Random Nerd Tutorials](https://randomnerdtutorials.com/esp32-dc-motor-l298n-motor-driver-control-speed-direction/) — L298N wiring + LEDC speed/direction.
- [Interface L298N with ESP32 — Microcontrollerslab](https://microcontrollerslab.com/l298n-dc-motor-driver-module-esp32-tutorial/) — ENA/ENB PWM, IN pin direction logic.
- [PID Tuning for Line Follower — ThinkRobotics](https://thinkrobotics.com/blogs/learn/pid-tuning-for-line-follower-complete-how-to-guide) — Kp→Kd→Ki tuning method.
