# Line Follower Robot Code

Fast line-following robot for **MECC 2026** — ESP32 + dual H-bridge motor driver + 5 digital IR
sensors.

The robot steers with a **PD controller** running a **1 kHz** control loop. All 5 sensors are
read in a single direct GPIO register access (`GPIO.in`) instead of five `digitalRead()` calls,
which leaves plenty of headroom for the fast loop.

## Sensor layout and turning logic

The five sensors are not all used for line following. Three in the middle track the line, and
the two on the outside watch for the black boxes on the track:

```
   left side                                             right side
      │                                                       │
    IR1 ─────── IR2 ── IR3 ── IR4 ─────── IR5
   LEFT BOX     mid-    centre   mid-    RIGHT BOX
     -4         left     0      right       +4
                -2               +2
```

| Rule | Behaviour |
|---|---|
| **IR3 (centre)** sees the line | robot follows it straight |
| Line drifts under **IR2 / IR4** | PD controller steers back to centre |
| **IR1 (left box)** sees a box | the **next decision steers LEFT** |
| **IR5 (right box)** sees a box | the **next decision steers RIGHT** |
| **All three middle sensors** dark (thick box) | the robot **TURNS** |
| No middle sensor sees the line | search turn toward `lastSide`, stop after 3 s |

A box is never allowed to pull the line error: the outer sensors contribute a separate,
short-lived steering kick (`boxKick`) that fades out over `BOX_HOLD_MS`, so the robot shifts
sideways and then settles back onto the line.

A thick box (all three middle sensors dark) triggers a pivot turn for `turnMs` at `turnSpeed`,
toward the box that was just detected, otherwise toward the direction the line was already
curving. After the turn the robot resumes following the line, and the same mark is ignored for
`TURN_REARM_MS` so it cannot trigger twice.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `Kp` | 40 | Proportional gain — how hard it corrects |
| `Kd` | 60 | Derivative gain — damping against wobble |
| `baseSpeed` | 240 | PWM on straights (0–255) |
| `slowK` | 20 | How much it slows down in corners |
| `minSpeed` | 120 | Lowest PWM while tracking |
| `pivotSpeed` | 180 | PWM used to spin and re-find the line |
| `boxKick` | 80 | Steering kick added when a side sensor sees a box |
| `turnSpeed` | 170 | PWM for the thick-box pivot turn |
| `turnMs` | 450 | Duration of the thick-box pivot turn |

## Hardware

- ESP32 Dev Module (ESP32-WROOM-32)
- Dual H-bridge motor driver board (2 direction pins + 1 PWM pin per motor)
- 5 × digital IR sensors (TCRT5000 style, **active-low**: LOW = dark surface)
- 2 × DC gear motors, battery pack

### Wiring

| Function | ESP32 GPIO |
|---|---|
| IR1 — left box sensor | 23 |
| IR2 — line, middle left | 22 |
| IR3 — line, centre | 21 |
| IR4 — line, middle right | 19 |
| IR5 — right box sensor | 18 |
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
b 80      set box steer kick
u 170     set thick-box turn speed
w 450     set thick-box turn duration (ms)
v         print current parameters
t         wheel test
```

If it **cuts corners**, increase `Kp`. If it **oscillates/zig-zags**, increase `Kd`.
If it flies off on fast corners, lower `baseSpeed` or raise `slowK`.
If it ignores the boxes, raise `boxKick`. If the box shift is too violent, lower it.

If the line is lost for 3 seconds the robot stops automatically.
