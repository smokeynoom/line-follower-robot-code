# Line Follower Robot Code

Fast line-following robot for **MECC 2026** — ESP32 + dual H-bridge motor driver + 5 digital IR
sensors.

The robot steers with a **PD controller** running a **1 kHz** control loop. All five sensors are
read in a single direct GPIO register access (`GPIO.in`) instead of five `digitalRead()` calls,
so the fast loop costs almost nothing. Build: **284,099 bytes (21% of flash)**, 22,392 bytes of
RAM.

## How it drives

The five sensors do two different jobs. Three in the middle track the line; the two on the
outside watch for the black boxes on the track:

```
        left                                             right
   ┌────────────┐                                  ┌────────────┐
   │ IR1  GPIO23│                                  │ IR5  GPIO18│
   │ BOX sensor │                                  │ BOX sensor │
   └────────────┘                                  └────────────┘
                  ┌────────┐ ┌────────┐ ┌────────┐
                  │  IR2   │ │  IR3   │ │  IR4   │
                  │  GPIO22│ │  GPIO21│ │  GPIO19│
                  └────────┘ └────────┘ └────────┘
                    mid-left   centre    mid-right
                    weight -2  weight 0  weight +2
```

| Rule | Behaviour |
|---|---|
| **IR3 (centre)** sees the line | follow it straight |
| Line drifts under **IR2 / IR4** | PD controller steers back to centre |
| **IR1 (left box)** sees a box | the **next decision steers LEFT** |
| **IR5 (right box)** sees a box | the **next decision steers RIGHT** |
| Box on both sides at once | the two cancel, no bias |
| **All three middle sensors** dark (thick box) | the robot **TURNS** |
| No middle sensor sees the line | search turn toward `lastSide`, stop after 3 s |

The outer sensors never enter the line error — a box must not drag the robot off the line.
Instead each box sets a **latched steering bias** that lands on the next decision and fades out
over `BOX_HOLD_MS`, so the robot actually shifts sideways instead of twitching for one 1 ms
cycle. The kick is `boxKick` PWM units, added on top of the PD correction.

A **thick box** (all three middle sensors dark) suspends line following: the robot pivots for
`turnMs` at `turnSpeed`, turning toward the box it just passed, else the way the line was
already curving, else right. Then it re-acquires the line, and the same mark is ignored for
`TURN_REARM_MS` so one box cannot trigger two turns.

## Parameters you can change over serial

| Parameter | Default | Command | Meaning |
|---|---|---|---|
| `Kp` | 40 | `p` | Proportional gain — how hard it corrects |
| `Kd` | 60 | `d` | Derivative gain — damping against wobble |
| `baseSpeed` | 240 | `s` | PWM on straights (0–255) |
| `slowK` | 20 | `k` | How much it slows down in corners |
| `minSpeed` | 120 | — | Lowest PWM while tracking |
| `pivotSpeed` | 180 | `m` | PWM used to spin and re-find the line |
| `boxKick` | 80 | `b` | Steering kick when a side sensor sees a box |
| `turnSpeed` | 170 | `u` | PWM for the thick-box pivot turn |
| `turnMs` | 450 | `w` | Duration of the thick-box pivot turn (ms) |

## Fixed constants (change in the sketch, re-upload)

| Constant | Value | Meaning |
|---|---|---|
| `CONTROL_US` | 1000 | Control period — 1 ms, i.e. 1 kHz |
| `BOX_HOLD_MS` | 120 | How long a box steer kick lasts |
| `TURN_REARM_MS` | 300 | Cooldown before a thick box can trigger again |
| `LOST_STOP_MS` | 3000 | Stop after the line has been lost this long |
| `MIN_PWM` | 60 | Dead-zone floor so motors actually turn |
| `PWM_FREQ` | 5000 | Motor PWM frequency (Hz), 8-bit resolution |

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
- Open the Serial Monitor at **115200 baud**. `v` prints both parameter lines; the startup line
  `Sensors  Lbox:0  line:010  Rbox:0` shows what each sensor sees right now — hand-test it by
  sliding the robot over the line and over a box before every run.

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
t         wheel test (robot must be stopped)
```

Tuning, in the order it matters:

- Cuts corners → raise `Kp`. Zig-zags → raise `Kd`.
- Flies off on fast corners → lower `baseSpeed` or raise `slowK`.
- Ignores the boxes → raise `boxKick`; shifts too violently → lower it.
- Overturns on junctions or manhole-sized marks → lower `w` (try 250) or `u` (try 140).

## First-run checklist

1. `t` over serial with the robot lifted — both wheels turn the right way. Fix
   `LEFT_INVERT` / `RIGHT_INVERT` if not.
2. Slide the robot across the line by hand and watch `Sensors ... line:...` — the middle bits
   must change. If nothing changes, check the sensor polarity and wiring.
3. Slide a box past each outer sensor — `Lbox:` / `Rbox:` must flip to 1.
4. Only then press **BOOT** and let it run, starting with `baseSpeed` around 150 and raising it.

## Repo contents

| File | Purpose |
|---|---|
| `FastLineFollower.ino` | the sketch |
| `README.md` | this document |
| `ESP32_LineFollower_Setup.pdf` | printable IDE setup guide with troubleshooting |

## Known limits

- The gains are first guesses for the box behaviour — they have not been tested on a track yet.
- The thick-box rule is defined purely as "three middle sensors dark", so any wide black region
  fires it, not only actual boxes.
- Detection distance is whatever the IR modules give you; they see the surface, not obstacles
  standing up.
