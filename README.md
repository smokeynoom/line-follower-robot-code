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
| **IR1 (left box)** passes a box | the **next turn goes LEFT** |
| **IR5 (right box)** passes a box | the **next turn goes RIGHT** |
| Both outer sensors dark at once | crossing line, ignored |
| **All three middle sensors** dark (thick box) | creep forward, then **spin until IR3 finds the line** |
| No middle sensor sees the line | hard arc toward `lastSide` for 150 ms, then spin in place, stop after 3 s |

The outer sensors never enter the line error, so a box can't drag the robot off the line.
A side box counts **once it has passed**: one outer sensor dark for at least 3 ms, then bright
again. The direction it gives is **remembered until the next thick box uses it**, with no time
limit. The newest box wins. If the middle three went all dark during that stretch, it was a
wide thick box and not a side box, so it is ignored.

A **thick box** has to hold all three middle sensors dark for 3 ms (so one speck can't fire it).
Then the robot:

1. creeps forward `creepMs` so the wheels, not just the sensors, are over the junction
2. spins toward the remembered side (else the way the line was curving, else right) **until IR3
   leaves the dark and finds the line again**, so it doesn't depend on battery level
   (`turnMs` is only a timeout)
3. ignores every box for `TURN_REARM_MS`, so the spin sweeping over marks can't queue a
   bogus turn

### Variants to try on the track

| Switch | What it does |
|---|---|
| `y 0` (default) | side box only picks the next turn, robot stays on the line |
| `y 1` | also nudges toward the box right away (`boxKick`), like the first version |
| `c 0` | no creep: spin the moment the sensors hit the box (sensors far from the axle) |
| `c 60`–`c 150` | creep longer if it turns too early and cuts the junction |

## Parameters you can change over serial

| Parameter | Default | Command | Meaning |
|---|---|---|---|
| `Kp` | 40 | `p` | Proportional gain — how hard it corrects |
| `Kd` | 60 | `d` | Derivative gain — damping against wobble |
| `baseSpeed` | 240 | `s` | PWM on straights (0–255) |
| `slowK` | 20 | `k` | How much it slows down in corners |
| `minSpeed` | 120 | — | Lowest PWM while tracking |
| `pivotSpeed` | 180 | `m` | PWM used to spin and re-find the line |
| `nudge` | 0 | `y` | 1 = also steer toward a side box right away |
| `boxKick` | 80 | `b` | Strength of that nudge |
| `turnSpeed` | 170 | `u` | PWM for the thick-box creep and spin |
| `creepMs` | 60 | `c` | Drive forward this long before spinning (ms) |
| `turnMs` | 1200 | `w` | Give up on the spin after this long (ms) |

## Fixed constants (change in the sketch, re-upload)

| Constant | Value | Meaning |
|---|---|---|
| `CONTROL_US` | 1000 | Control period — 1 ms, i.e. 1 kHz |
| `BOX_HOLD_MS` | 120 | How long the optional nudge lasts |
| `MARK_MIN_SAMPLES` | 3 | Side box must stay dark this many ms to count |
| `THICK_CONFIRM` | 3 | Thick box must stay dark this many ms to fire |
| `LOST_ARC_MS` | 150 | Arc toward a lost line this long before spinning |
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
y 1       box nudge on (y 0 = off)
b 80      set nudge strength
u 170     set thick-box turn speed
c 60      set creep before the turn (ms)
w 1200    set thick-box turn timeout (ms)
v         print current parameters
t         wheel test (robot must be stopped)
```

Tuning, in the order it matters:

- Cuts corners → raise `Kp`. Zig-zags → raise `Kd`.
- Flies off on fast corners → lower `baseSpeed` or raise `slowK`.
- Turns the wrong way → `v` shows `pending=` (-1 left, 1 right); slide a box past each side
  sensor by hand and check it changes.
- Turns before reaching the junction → raise `c`. Overshoots past it → lower `c`.
- Spins past the new line → lower `u` (try 140).

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
| `test/sim.cpp` | runs the box / turn logic on a PC: `g++ -std=c++17 -Itest/stub -x c++ test/sim.cpp -o sim && ./sim` |

## Known limits

- The gains, `creepMs` and `turnSpeed` are first guesses. Only the logic is tested (in the PC
  simulator), not a real track.
- A thick box with no side box before it turns toward the last curve direction. If the track
  means "go straight" there, that needs a rule change.
- The thick-box rule is defined purely as "three middle sensors dark", so any wide black region
  fires it, not only actual boxes.
- Detection distance is whatever the IR modules give you; they see the surface, not obstacles
  standing up.
