/*
  MECC 2026 - Fast Line Follower

  Board : ESP32 Dev Module
  Motors: 2x DC gear motors via a dual H-bridge driver board
  Sensors: 5x digital IR modules

  IMPORTANT:
  This version uses the ESP32 Arduino core 3.x PWM API:
    ledcAttach()
    ledcWrite()

  BOOT button (GPIO0):
    Press once  -> START
    Press again -> STOP

  Serial Monitor @ 115200:

    p 30    -> Set Kp
    d 25    -> Set Kd
    s 210   -> Set base speed
    f 255   -> Set straight boost speed
    z 15    -> Set curve brake (ms, 0 = off)
    k 25    -> Set corner slow-down
    m 170   -> Set pivot/search speed
    b 70    -> Set box steer kick
    u 170   -> Set thick-box turn speed
    w 1200  -> Set thick-box turn timeout (ms)
    c 60    -> Set forward creep before the turn (ms)
    y 0/1   -> Box nudge off / on
    v       -> Show current parameters
    t       -> Wheel test

  SENSOR LAYOUT (left to right as seen from behind the robot):

    IR1 = LEFT  BOX sensor   (GPIO23)  looks off the line, left side
    IR2 = line, middle LEFT  (GPIO22)
    IR3 = line, CENTRE       (GPIO21)  the sensor that tracks the line
    IR4 = line, middle RIGHT (GPIO19)
    IR5 = RIGHT BOX sensor   (GPIO18)  looks off the line, right side

  TURNING LOGIC:
    * IR3 follows the line; IR2 / IR4 help as soon as it drifts.
    * LEFT box sensor passes a box  -> the next turn goes LEFT.
    * RIGHT box sensor passes a box -> the next turn goes RIGHT.
      (Remembered until the turn uses it; the latest box wins.)
    * All 3 middle sensors dark (thick box) -> creep forward, then
      spin until IR3 finds the line again.
    * Anything else -> normal PD line following.

  Assumption:
    IR modules are ACTIVE-LOW:
      LOW  = dark / line or box detected
      HIGH = bright surface

  If robot steers AWAY from the line:
    FLIP_STEERING = true

  PERFORMANCE OPTIMISATIONS:
    * All 5 IR sensors are read in ONE direct GPIO register
      access (GPIO.in) instead of five digitalRead() calls.
      ~100x faster sensor read.
    * Control loop runs at 1 kHz (was 500 Hz).
    * PD gains retuned for higher-speed tracking.
*/

// ============================================================
// LIBRARY
// ============================================================

#include <Arduino.h>

// gpio_dev_t / GPIO — register-level GPIO struct shipped with the
// ESP32 core. GPIO.in holds GPIO0-31, so all five sensors are one load.
#include "soc/gpio_struct.h"

// ============================================================
// PIN DEFINITIONS
// ============================================================

// IR sensor GPIO pins (for pinMode / debug only)
// Actual reads use direct register access — see fastReadSensors()
const uint8_t IR_PINS[5] = {
  23, 22, 21, 19, 18
};

// ============================================================
// DIRECT GPIO REGISTER MASKS  (fast sensor read)
// ============================================================

// All five sensors sit on GPIO18-23, which live in the low input
// register (GPIO.in), so ALL FIVE are read in ONE 32-bit load.
// Bit N of GPIO.in = physical GPIO N.
//
//   IR1 (GPIO23) -> bit 23   1 << 23    LEFT  BOX
//   IR2 (GPIO22) -> bit 22   1 << 22    line, mid-left
//   IR3 (GPIO21) -> bit 21   1 << 21    line, centre
//   IR4 (GPIO19) -> bit 19   1 << 19    line, mid-right
//   IR5 (GPIO18) -> bit 18   1 << 18    RIGHT BOX
//
// Each mask is written next to its own pin number, so a sensor can
// never end up wired to the wrong bit.
static const uint32_t M_IR1 = (1u << 23);
static const uint32_t M_IR2 = (1u << 22);
static const uint32_t M_IR3 = (1u << 21);
static const uint32_t M_IR4 = (1u << 19);
static const uint32_t M_IR5 = (1u << 18);

static const uint32_t SENSOR_MASK =
  M_IR1 | M_IR2 | M_IR3 | M_IR4 | M_IR5;

// Line-following weights — MIDDLE sensors only.
// The two outer sensors detect boxes and never contribute to the
// line error, otherwise a box would drag the robot off the line.
static const float W_MID_LEFT  = -2.0f;   // IR2
static const float W_CENTRE    =  0.0f;   // IR3
static const float W_MID_RIGHT =  2.0f;   // IR4

// Left motor direction pins
const int IN1 = 27;
const int IN2 = 26;

// Right motor direction pins
const int IN3 = 13;
const int IN4 = 25;

// Motor enable / PWM pins
const int ENA = 14;
const int ENB = 33;

// BOOT button
const int BTN = 0;

// ============================================================
// HARDWARE OPTIONS
// ============================================================

const bool LEFT_INVERT   = false;
const bool RIGHT_INVERT  = false;
const bool FLIP_STEERING = false;

// IR sensor polarity
// ACTIVE-LOW:
// LOW = line detected
// HIGH = no line
const int LINE_DETECTED = LOW;

// PWM settings
const uint32_t PWM_FREQ = 5000;
const uint8_t PWM_RESOLUTION = 8;

// Minimum PWM before motor overcomes dead zone
const int MIN_PWM = 60;

// ============================================================
// TUNABLE PARAMETERS
// ============================================================

// PD gains — retuned for 1 kHz loop and higher speed.
// Increase Kp if robot cuts corners; increase Kd if it oscillates.
float Kp = 40.0f;
float Kd = 25.0f;

int baseSpeed = 240;

// Straights: once IR3 alone has seen the line for BOOST_AFTER_MS,
// speed ramps from baseSpeed up to boostSpeed over BOOST_RAMP_MS.
// For a fast run lower baseSpeed (corner-safe) and keep boost high.
int boostSpeed = 255;
const uint32_t BOOST_AFTER_MS = 100;
const uint32_t BOOST_RAMP_MS = 300;

// Leaving a boosted straight: reverse both motors this long to
// shed speed before the curve. 0 = off. Try 10-30.
int brakeMs = 0;

// Speed reduction for larger line error (lower = less slow-down)
float slowK = 20.0f;

// Minimum speed while line is visible
int minSpeed = 120;

// Search/pivot speed when line is lost
int pivotSpeed = 180;

// ------------------------------------------------------------
// BOX PARAMETERS
// ------------------------------------------------------------

// A side box sets the direction of the NEXT thick-box turn and is
// remembered until that turn uses it (no time limit).
//
// nudge = 1 also gives an immediate steering kick toward the box,
// like the first version did. nudge = 0 (default) stays on the line.
int nudge = 0;

// How hard the optional nudge steers (added to the PD correction)
float boxKick = 80.0f;

// How long the optional nudge lasts, fading out
const uint32_t BOX_HOLD_MS = 120;

// A side box only counts if its sensor stays dark this many
// consecutive 1 ms samples (rejects specks and noise)
const uint8_t MARK_MIN_SAMPLES = 3;

// Thick box = all three middle sensors dark for THICK_CONFIRM
// consecutive samples. The robot then:
//   1. creeps forward creepMs so the wheels reach the junction
//   2. spins at turnSpeed until IR3 leaves the dark and finds the
//      line again, giving up after turnMs
int turnSpeed = 170;
int creepMs = 60;
int turnMs = 1200;

const uint8_t THICK_CONFIRM = 3;

// Once a turn is done, ignore boxes for this long so the same
// mark cannot trigger a second turn
const uint32_t TURN_REARM_MS = 300;

// ============================================================
// CONTROL SETTINGS
// ============================================================

// The D term compares the error with the error D_WINDOW samples
// ago (32 ms at 1 kHz). On/off sensors change in jumps, so a
// sample-to-sample derivative is a 1 ms spike the motors never feel;
// over 32 ms it lasts long enough to damp the wobble. Power of two.
const uint8_t D_WINDOW = 32;

// 1000 microseconds = 1 ms = 1000 Hz
// 2x faster reaction vs previous 500 Hz — sensor reads
// are now fast enough to keep up.
const uint32_t CONTROL_US = 1000;

// Stop after line is lost for this long
const uint32_t LOST_STOP_MS = 3000;

// When the line slips off a side, first keep driving in a hard arc
// toward it for this long (keeps speed), and only then spin in place
const uint32_t LOST_ARC_MS = 150;

// Button debounce
const uint32_t BUTTON_DEBOUNCE_MS = 400;

// ============================================================
// STATE VARIABLES
// ============================================================

bool running = false;

// Recent errors for the D term (ring buffer)
int8_t errHist[D_WINDOW];
uint8_t errIdx = 0;

// Straight boost / curve brake
uint32_t centredSince = 0;
int lastSpeed = 0;
uint32_t brakeUntil = 0;

// -1 = line last seen on left
// +1 = line last seen on right
//  0 = no previous direction
int lastSide = 0;

uint32_t lostSince = 0;
uint32_t lastCtrl = 0;
uint32_t lastButtonPress = 0;

// ------------------------------------------------------------
// BOX STATE
// ------------------------------------------------------------

// Direction for the next thick-box turn: -1 left, +1 right, 0 none
int pendingTurn = 0;

// Side box tracking: dark-sample count, and whether this dark
// stretch was really something else (a wide box / crossing line)
uint8_t leftRun = 0;
uint8_t rightRun = 0;
bool leftSpoiled = false;
bool rightSpoiled = false;

// Optional nudge
int nudgeDir = 0;
uint32_t nudgeUntil = 0;

// Thick-box turn
enum Phase { FOLLOW, CREEP, SPIN };
Phase phase = FOLLOW;
int turnDir = 0;
uint32_t phaseUntil = 0;
bool spinLeftDark = false;   // IR3 has seen white since the spin began
uint8_t thickRun = 0;
uint32_t turnRearmAt = 0;

// ============================================================
// MOTOR CONTROL
// ============================================================

void driveOne(
  int in_a,
  int in_b,
  int pwmPin,
  int speed,
  bool invert
) {
  // Reverse motor direction if required
  if (invert) {
    speed = -speed;
  }

  // Limit speed to -255 ... +255
  int mag = constrain(abs(speed), 0, 255);

  // Apply minimum usable PWM
  if (mag > 0 && mag < MIN_PWM) {
    mag = MIN_PWM;
  }

  // Set direction
  if (speed > 0) {

    // Forward
    digitalWrite(in_a, HIGH);
    digitalWrite(in_b, LOW);

  }
  else if (speed < 0) {

    // Reverse
    digitalWrite(in_a, LOW);
    digitalWrite(in_b, HIGH);

  }
  else {

    // Stop
    digitalWrite(in_a, LOW);
    digitalWrite(in_b, LOW);

    mag = 0;
  }

  // Write PWM duty cycle
  ledcWrite(pwmPin, mag);
}


// Positive = forward
// Negative = reverse
void setMotors(int left, int right) {

  driveOne(
    IN1,
    IN2,
    ENA,
    left,
    LEFT_INVERT
  );

  // Right motor uses mirrored direction pins
  driveOne(
    IN4,
    IN3,
    ENB,
    right,
    RIGHT_INVERT
  );
}


void stopMotors() {
  setMotors(0, 0);
}


// Spin in place. dir > 0 = turn RIGHT, dir < 0 = turn LEFT
void pivot(int dir) {

  if (dir > 0) {
    setMotors(turnSpeed, -turnSpeed);
  }
  else {
    setMotors(-turnSpeed, turnSpeed);
  }
}

// ============================================================
// SENSOR READING  (direct GPIO register — no digitalRead)
// ============================================================

/*
  Reads all 5 IR sensors in a SINGLE GPIO register access.

  Returns:
    Number of MIDDLE sensors that see the line (0..3)

  error:
    Weighted position of the line under the three middle sensors

      IR2 (mid-LEFT)  -> -2
      IR3 (CENTRE)    ->  0
      IR4 (mid-RIGHT) -> +2

  leftBox / rightBox:
    true when the matching outer sensor sees a box

  The two outer sensors are NOT part of the line error — they only
  report boxes, which the main loop turns into a steering kick.
*/

__attribute__((always_inline))
int readSensors(float &error, bool &leftBox, bool &rightBox, bool &centre) {

  // ONE register read replaces five digitalRead() calls
  const uint32_t gpio = GPIO.in;

  // Active-LOW: a sensor reads 0 when it sees dark
  const uint32_t active = (~gpio) & SENSOR_MASK;

  leftBox  = (active & M_IR1) != 0;
  rightBox = (active & M_IR5) != 0;
  centre   = (active & M_IR3) != 0;

  int   count = 0;
  float sum   = 0.0f;

  if (active & M_IR2) { sum += W_MID_LEFT;  count++; }
  if (active & M_IR3) { sum += W_CENTRE;    count++; }
  if (active & M_IR4) { sum += W_MID_RIGHT; count++; }

  error = (count > 0) ? (sum / (float)count) : 0.0f;

  if (FLIP_STEERING) {
    error = -error;
  }

  return count;
}

// Forget the error history (start, and after a turn) so the D term
// does not kick on stale data
void resetD() {
  memset(errHist, 0, sizeof(errHist));
  errIdx = 0;
}

// ============================================================
// SIDE BOX TRACKING
// ============================================================

/*
  A side box counts once it has PASSED: one outer sensor dark for at
  least MARK_MIN_SAMPLES, then bright again. If the middle trio went
  all-dark, or the other outer sensor was dark too, during that
  stretch, it was a wide thick box or a crossing line, not a side box.
*/
void trackSideBox(
  bool dark,
  bool otherDark,
  int midCount,
  uint8_t &run,
  bool &spoiled,
  int dir,
  uint32_t nowMs
) {
  if (dark) {
    if (run < 255) run++;
    if (midCount >= 3 || otherDark) spoiled = true;
    return;
  }

  if (run >= MARK_MIN_SAMPLES && !spoiled) {
    pendingTurn = dir;
    nudgeDir = dir;
    nudgeUntil = nowMs + BOX_HOLD_MS;
  }

  run = 0;
  spoiled = false;
}

// ============================================================
// SENSOR DEBUG
// ============================================================

void printSensors() {

  bool leftBox  = (digitalRead(IR_PINS[0]) == LINE_DETECTED);
  bool midLeft  = (digitalRead(IR_PINS[1]) == LINE_DETECTED);
  bool centre   = (digitalRead(IR_PINS[2]) == LINE_DETECTED);
  bool midRight = (digitalRead(IR_PINS[3]) == LINE_DETECTED);
  bool rightBox = (digitalRead(IR_PINS[4]) == LINE_DETECTED);

  Serial.printf(
    "Sensors  Lbox:%d  line:%d%d%d  Rbox:%d\n",
    leftBox,
    midLeft,
    centre,
    midRight,
    rightBox
  );
}

// ============================================================
// WHEEL TEST
// ============================================================

void wheelTest() {

  Serial.println();
  Serial.println("==============================");
  Serial.println("WHEEL TEST");
  Serial.println("Lift the robot off the ground");
  Serial.println("==============================");

  // Left forward
  Serial.println("Left forward");
  setMotors(150, 0);
  delay(1500);

  stopMotors();
  delay(300);

  // Left reverse
  Serial.println("Left reverse");
  setMotors(-150, 0);
  delay(1500);

  stopMotors();
  delay(300);

  // Right forward
  Serial.println("Right forward");
  setMotors(0, 150);
  delay(1500);

  stopMotors();
  delay(300);

  // Right reverse
  Serial.println("Right reverse");
  setMotors(0, -150);
  delay(1500);

  stopMotors();

  Serial.println();
  Serial.println("Wheel test complete.");
  Serial.println("Change LEFT_INVERT or RIGHT_INVERT");
  Serial.println("if a motor spins the wrong way.");
  Serial.println();
}

// ============================================================
// PRINT PARAMETERS
// ============================================================

void printParams() {

  Serial.printf(
    "Kp=%.1f  Kd=%.1f  base=%d  boost=%d  brakeMs=%d  slowK=%.1f  min=%d  pivot=%d\n",
    Kp,
    Kd,
    baseSpeed,
    boostSpeed,
    brakeMs,
    slowK,
    minSpeed,
    pivotSpeed
  );

  Serial.printf(
    "nudge=%d  boxKick=%.1f  turnSpeed=%d  creepMs=%d  turnMs=%d  pending=%d\n",
    nudge,
    boxKick,
    turnSpeed,
    creepMs,
    turnMs,
    pendingTurn
  );
}

// ============================================================
// SERIAL COMMANDS
// ============================================================

void handleSerial() {

  if (!Serial.available()) {
    return;
  }

  char command = Serial.read();

  // Ignore newline / carriage return / spaces
  if (
    command == '\n' ||
    command == '\r' ||
    command == ' '
  ) {
    return;
  }

  // ----------------------------------------------------------
  // Commands without a number
  // ----------------------------------------------------------

  if (command == 'v') {

    printParams();
    return;
  }

  if (command == 't') {

    if (!running) {
      wheelTest();
    }
    else {
      Serial.println("Stop the robot before wheel test.");
    }

    return;
  }

  // ----------------------------------------------------------
  // Commands requiring a number
  // ----------------------------------------------------------

  float value = Serial.parseFloat();

  switch (command) {

    case 'p':
      Kp = value;
      Serial.println("Kp updated.");
      break;

    case 'd':
      Kd = value;
      Serial.println("Kd updated.");
      break;

    case 's':
      baseSpeed = constrain((int)value, 0, 255);
      Serial.println("Base speed updated.");
      break;

    case 'k':
      slowK = value;
      Serial.println("Slow-down factor updated.");
      break;

    case 'm':
      pivotSpeed = constrain((int)value, 0, 255);
      Serial.println("Pivot speed updated.");
      break;

    case 'b':
      boxKick = value;
      Serial.println("Box steer kick updated.");
      break;

    case 'u':
      turnSpeed = constrain((int)value, 0, 255);
      Serial.println("Thick-box turn speed updated.");
      break;

    case 'w':
      turnMs = constrain((int)value, 0, 2000);
      Serial.println("Thick-box turn timeout updated.");
      break;

    case 'f':
      boostSpeed = constrain((int)value, 0, 255);
      Serial.println("Boost speed updated.");
      break;

    case 'z':
      brakeMs = constrain((int)value, 0, 100);
      Serial.println("Curve brake updated.");
      break;

    case 'c':
      creepMs = constrain((int)value, 0, 500);
      Serial.println("Creep before turn updated.");
      break;

    case 'y':
      nudge = (value != 0.0f) ? 1 : 0;
      Serial.println("Box nudge updated.");
      break;

    default:
      Serial.println("Unknown command.");
      Serial.println("Use: p, d, s, f, z, k, m, b, u, w, c, y, v, t");
      return;
  }

  printParams();
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  // ----------------------------------------------------------
  // Serial
  // ----------------------------------------------------------

  Serial.begin(115200);

  // ----------------------------------------------------------
  // IR sensors
  // ----------------------------------------------------------

  for (int i = 0; i < 5; i++) {
    pinMode(IR_PINS[i], INPUT_PULLUP);
  }

  // ----------------------------------------------------------
  // BOOT button
  // ----------------------------------------------------------

  pinMode(BTN, INPUT_PULLUP);

  // ----------------------------------------------------------
  // Motor direction pins
  // ----------------------------------------------------------

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // ----------------------------------------------------------
  // PWM setup
  // ESP32 Arduino core 3.x API
  // ----------------------------------------------------------

  ledcAttach(
    ENA,
    PWM_FREQ,
    PWM_RESOLUTION
  );

  ledcAttach(
    ENB,
    PWM_FREQ,
    PWM_RESOLUTION
  );

  // Make sure robot starts stopped
  stopMotors();

  // ----------------------------------------------------------
  // Startup messages
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("==============================");
  Serial.println("MECC 2026 FAST LINE FOLLOWER");
  Serial.println("==============================");
  Serial.println("Ready.");
  Serial.println("Press BOOT to start/stop.");
  Serial.println();

  printParams();
  printSensors();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // Serial control
  // ----------------------------------------------------------

  handleSerial();

  // ----------------------------------------------------------
  // START / STOP BUTTON
  // ----------------------------------------------------------

  uint32_t currentMillis = millis();

  if (
    digitalRead(BTN) == LOW &&
    (currentMillis - lastButtonPress >= BUTTON_DEBOUNCE_MS)
  ) {

    lastButtonPress = currentMillis;

    // Toggle running state
    running = !running;

    if (running) {

      // Give user time to remove hand
      delay(800);

      // Reset controller
      resetD();
      centredSince = 0;
      brakeUntil = 0;
      lostSince = 0;
      lastSide = 0;

      // Reset box handling
      pendingTurn = 0;
      leftRun = rightRun = 0;
      leftSpoiled = rightSpoiled = false;
      nudgeUntil = 0;
      phase = FOLLOW;
      thickRun = 0;
      turnRearmAt = millis();

      lastCtrl = micros();

      Serial.println("RUN");
    }
    else {

      stopMotors();

      Serial.println("STOP");
    }
  }

  // ----------------------------------------------------------
  // Robot stopped
  // ----------------------------------------------------------

  if (!running) {
    return;
  }

  // ----------------------------------------------------------
  // 1 kHz CONTROL LOOP
  // ----------------------------------------------------------

  uint32_t now = micros();

  if ((uint32_t)(now - lastCtrl) < CONTROL_US) {
    return;
  }

  lastCtrl = now;

  // ----------------------------------------------------------
  // READ SENSORS
  // ----------------------------------------------------------

  float error = 0.0f;
  bool  leftBox  = false;
  bool  rightBox = false;
  bool  centre   = false;

  int midCount = readSensors(error, leftBox, rightBox, centre);

  uint32_t nowMs = millis();

  // ==========================================================
  // THICK-BOX TURN IN PROGRESS
  // ==========================================================

  if (phase == CREEP) {

    // Roll forward so the wheels, not just the sensors, reach the box
    if ((int32_t)(nowMs - phaseUntil) < 0) {
      setMotors(turnSpeed, turnSpeed);
      return;
    }

    phase = SPIN;
    phaseUntil = nowMs + (uint32_t)turnMs;
    spinLeftDark = !centre;
  }

  if (phase == SPIN) {

    // Spin until IR3 has left the dark and found the line again.
    // Sensor-guided, so it does not depend on battery level.
    if (!centre) {
      spinLeftDark = true;
    }

    bool found = spinLeftDark && centre;

    if (!found && (int32_t)(nowMs - phaseUntil) < 0) {
      pivot(turnDir);
      return;
    }

    // Turn finished (or timed out - line-lost search takes over)
    phase = FOLLOW;
    turnRearmAt = nowMs + TURN_REARM_MS;
    lastSide = turnDir;
    resetD();
    centredSince = 0;
    lostSince = 0;
  }

  // ==========================================================
  // SIDE BOXES  -> remember the direction of the next turn
  // ==========================================================

  bool armed = (int32_t)(nowMs - turnRearmAt) >= 0;

  if (armed) {
    trackSideBox(leftBox, rightBox, midCount,
                 leftRun, leftSpoiled, -1, nowMs);
    trackSideBox(rightBox, leftBox, midCount,
                 rightRun, rightSpoiled, 1, nowMs);
  }
  else {
    // Just turned: ignore whatever the spin swept the sensors over
    leftRun = rightRun = 0;
    leftSpoiled = rightSpoiled = false;
  }

  // ==========================================================
  // THICK BOX  (all three middle sensors dark)  -> TURN
  // ==========================================================

  if (midCount >= 3 && armed) {
    if (thickRun < 255) thickRun++;
  }
  else {
    thickRun = 0;
  }

  if (thickRun >= THICK_CONFIRM) {

    // Turn toward the last side box, else the way the line was
    // already curving, else right
    if (pendingTurn != 0) {
      turnDir = pendingTurn;
    }
    else if (lastSide != 0) {
      turnDir = lastSide;
    }
    else {
      turnDir = 1;
    }

    pendingTurn = 0;
    nudgeUntil = 0;
    thickRun = 0;

    phase = CREEP;
    phaseUntil = nowMs + (uint32_t)creepMs;
    setMotors(turnSpeed, turnSpeed);
    return;
  }

  // ==========================================================
  // LINE LOST
  // ==========================================================

  if (midCount == 0) {

    if (lostSince == 0) {
      lostSince = nowMs;
    }

    uint32_t lostFor = nowMs - lostSince;

    if (lostFor > LOST_STOP_MS) {

      stopMotors();
      running = false;
      Serial.println("LINE LOST - STOPPED");
      return;
    }

    // The line slipped off one side: arc hard toward it first, as if
    // it were one sensor further out than IR2 / IR4
    if (lostFor < LOST_ARC_MS && lastSide != 0) {

      error = 3.0f * lastSide;
    }
    else {

      // Still lost: spin in place toward where the line was
      if (lastSide < 0) {
        setMotors(-pivotSpeed, pivotSpeed);
      }
      else {
        setMotors(pivotSpeed, -pivotSpeed);
      }

      return;
    }
  }
  else {

    lostSince = 0;
  }

  // ==========================================================
  // LINE FOUND
  // ==========================================================

  // Remember which side line was last seen
  if (error > 0.5f) {

    lastSide = 1;

  }
  else if (error < -0.5f) {

    lastSide = -1;
  }

  // ==========================================================
  // PD CONTROL
  // ==========================================================

  // Proportional term
  float proportional = error;

  // Derivative term: change over the last D_WINDOW ms
  float derivative = error - errHist[errIdx];
  errHist[errIdx] = (int8_t)error;
  errIdx = (errIdx + 1) & (D_WINDOW - 1);

  // Calculate steering correction
  float correction =
    (Kp * proportional) +
    (Kd * derivative);

  // Optional nudge toward a side box, fading out over BOX_HOLD_MS
  if (nudge && (int32_t)(nudgeUntil - nowMs) > 0) {

    float weight =
      (float)(nudgeUntil - nowMs) / (float)BOX_HOLD_MS;

    correction += nudgeDir * boxKick * weight;
  }

  // ==========================================================
  // SPEED CONTROL
  // ==========================================================

  // Fast on straight sections
  // Slower when error becomes larger
  int speed =
    baseSpeed -
    (int)(slowK * fabsf(error));

  // Do not go below minimum speed
  if (speed < minSpeed) {
    speed = minSpeed;
  }

  // Straight boost: IR3 alone on the line -> ramp up to boostSpeed
  if (midCount == 1 && centre) {

    if (centredSince == 0) {
      centredSince = nowMs;
    }

    uint32_t onStraight = nowMs - centredSince;

    if (onStraight > BOOST_AFTER_MS && boostSpeed > speed) {

      uint32_t t = onStraight - BOOST_AFTER_MS;

      if (t > BOOST_RAMP_MS) {
        t = BOOST_RAMP_MS;
      }

      speed += (int)((boostSpeed - speed) * t / BOOST_RAMP_MS);
    }
  }
  else {

    // Coming off a boosted straight: brake before the curve
    if (centredSince != 0 && brakeMs > 0 && lastSpeed > baseSpeed + 5) {
      brakeUntil = nowMs + (uint32_t)brakeMs;
    }

    centredSince = 0;
  }

  lastSpeed = speed;

  if ((int32_t)(brakeUntil - nowMs) > 0) {
    setMotors(-255, -255);
    return;
  }

  // ==========================================================
  // DIFFERENTIAL STEERING
  // ==========================================================

  /*
    Positive error = line is RIGHT

    Therefore:
      LEFT  wheel -> faster
      RIGHT wheel -> slower

    Negative error = line is LEFT

    Therefore:
      LEFT  wheel -> slower
      RIGHT wheel -> faster
  */

  int left =
    constrain(
      (int)(speed + correction),
      -255,
      255
    );

  int right =
    constrain(
      (int)(speed - correction),
      -255,
      255
    );

  // Send speeds to motors
  setMotors(left, right);
}