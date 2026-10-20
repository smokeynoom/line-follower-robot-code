# Line Follower Robot Code

Fast line-following robot for **MECC 2026** — ESP32 + L298N dual H-bridge + 5 digital IR sensors.

The robot steers with a **PD controller** running a **1 kHz** control loop. All 5 sensors are
read in a single direct GPIO register access (`GPIO.in1`) instead of five `digitalRead()`
calls, which leaves plenty of headroom for the fast loop.

| Parameter | Default | Meaning |
|---|---|---|
| `Kp` | 40 | Proportional gain — how hard it corrects |
| `Kd` | 60 | Derivative gain — damping against wobble |
| `baseSpeed` | 240 | PWM on straights (0–255) |
| `slowK` | 20 | How much it slows down in corners |
| `minSpeed` | 120 | Lowest PWM while tracking |
| `pivotSpeed` | 180 | PWM used to spin and re-find the line |

## Hardware

- ESP32 Dev Module (ESP32-WROOM-32)
- L298N (or equivalent) dual H-bridge motor driver
- 5 × digital IR line sensors (TCRT5000 style, **active-low**: LOW = line)
- 2 × DC gear motors, battery pack

### Wiring

| Function | ESP32 GPIO |
|---|---|
| IR1 (far left) | 23 |
| IR2 (left) | 22 |
| IR3 (center) | 21 |
| IR4 (right) | 19 |
| IR5 (far right) | 18 |
| Motor A direction IN1 / IN2 | 27 / 26 |
| Motor B direction IN3 / IN4 | 13 / 25 |
| Motor A PWM ENA | 14 |
| Motor B PWM ENB | 33 |
| BOOT button (start/stop) | 0 |

If the robot steers **away** from the line, flip `FLIP_STEERING` in the sketch.
If one wheel spins backwards, flip `LEFT_INVERT` or `RIGHT_INVERT`.

## Setup

1. Install **Arduino IDE 2.x** and the **CP210x USB-to-UART driver** (the ESP32 board's USB
   bridge chip).
2. Add this URL in *File → Preferences → Additional boards manager URLs*:
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
3. In Boards Manager install **"esp32 by Espressif Systems"** — *not* "Arduino ESP32 Boards".
   This sketch needs the core 3.x PWM API (`ledcAttach`).
4. Select **Tools → Board → esp32 → ESP32 Dev Module** and the matching COM port.
5. Open `FastLineFollower.ino`, then **Upload**.

A printable walkthrough with troubleshooting is included:
[ESP32_LineFollower_Setup.pdf](ESP32_LineFollower_Setup.pdf).

## Running

- Press **BOOT** once to start, again to stop.
- Open the Serial Monitor at **115200 baud** to see telemetry and tune live:

```
p 45      set Kp
d 55      set Kd
s 230     set base speed
k 18      set corner slow-down
m 170     set pivot/search speed
v         print current parameters
t         wheel test
```

If it **cuts corners**, increase `Kp`. If it **oscillates/zig-zags**, increase `Kd`.
If it flies off on fast corners, lower `baseSpeed` or raise `slowK`.

If the line is lost for 3 seconds the robot stops automatically.
