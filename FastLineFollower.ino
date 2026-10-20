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
    d 80    -> Set Kd
    s 210   -> Set base speed
    k 25    -> Set corner slow-down
    m 170   -> Set pivot/search speed
    b 70    -> Set box steer kick
    u 170   -> Set thick-box turn speed
    w 450   -> Set thick-box turn duration (ms)
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
    * LEFT box sensor sees a box  -> next decision steers LEFT.
    * RIGHT box sensor sees a box -> next decision steers RIGHT.
    * All 3 middle sensors dark (thick box) -> the robot TURNS.
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
float Kd = 60.0f;

int baseSpeed = 240;

// Speed reduction for larger line error (lower = less slow-down)
float slowK = 20.0f;

// Minimum speed while line is visible
int minSpeed = 120;

// Search/pivot speed when line is lost
int pivotSpeed = 180;

// ------------------------------------------------------------
// BOX PARAMETERS
// ------------------------------------------------------------

// How hard the robot steers when a SIDE sensor sees a box
// (added to the PD correction, in PWM units)
float boxKick = 80.0f;

// How long that steering kick lasts after the box is seen.
// 120 ms at ~2 m/s pushes the robot a few cm sideways.
const uint32_t BOX_HOLD_MS = 120;

// Thick box = all three middle sensors dark. The robot pivots
// for turnMs at turnSpeed, then goes back to following the line.
int turnSpeed = 170;
int turnMs = 450;

// Once a thick box has been handled, ignore it for this long so
// the same mark cannot trigger a second turn
const uint32_t TURN_REARM_MS = 300;

// ============================================================
// CONTROL SETTINGS
// ============================================================

// 1000 microseconds = 1 ms = 1000 Hz
// 2x faster reaction vs previous 500 Hz — sensor reads
// are now fast enough to keep up.
const uint32_t CONTROL_US = 1000;

// Stop after line is lost for this long
const uint32_t LOST_STOP_MS = 3000;

// Button debounce
const uint32_t BUTTON_DEBOUNCE_MS = 400;

// ============================================================
// STATE VARIABLES
// ============================================================

bool running = false;

float lastError = 0.0f;
float dFilt = 0.0f;

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

// -1 = steer left, +1 = steer right, 0 = no box seen
float boxBias = 0.0f;
uint32_t boxBiasUntil = 0;

// Thick-box turn in progress
bool turning = false;
int turnDir = 0;
uint32_t turnUntil = 0;
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
int readSensors(float &error, bool &leftBox, bool &rightBox) {

  // ONE register read replaces five digitalRead() calls
  const uint32_t gpio = GPIO.in;

  // Active-LOW: a sensor reads 0 when it sees dark
  const uint32_t active = (~gpio) & SENSOR_MASK;

  leftBox  = (active & M_IR1) != 0;
  rightBox = (active & M_IR5) != 0;

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
    "Kp=%.1f  Kd=%.1f  base=%d  slowK=%.1f  min=%d  pivot=%d\n",
    Kp,
    Kd,
    baseSpeed,
    slowK,
    minSpeed,
    pivotSpeed
  );

  Serial.printf(
    "boxKick=%.1f  turnSpeed=%d  turnMs=%d\n",
    boxKick,
    turnSpeed,
    turnMs
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
      Serial.println("Thick-box turn duration updated.");
      break;

    default:
      Serial.println("Unknown command.");
      Serial.println("Use: p, d, s, k, m, b, u, w, v, t");
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
      lastError = 0.0f;
      dFilt = 0.0f;
      lostSince = 0;
      lastSide = 0;

      // Reset box handling
      boxBias = 0.0f;
      boxBiasUntil = 0;
      turning = false;
      turnDir = 0;
      turnUntil = 0;
      turnRearmAt = 0;

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

  int midCount = readSensors(error, leftBox, rightBox);

  uint32_t nowMs = millis();

  // ==========================================================
  // BOX SEEN BY A SIDE SENSOR
  // ==========================================================

  /*
    LEFT  box sensor sees a box -> steer LEFT
    RIGHT box sensor sees a box -> steer RIGHT

    The choice is LATCHED and applied to the next decision, then
    fades over BOX_HOLD_MS, so it actually moves the robot
    sideways instead of twitching for a single 1 ms cycle.
  */

  if (leftBox && !rightBox) {

    boxBias = -1.0f;                       // negative = left
    boxBiasUntil = nowMs + BOX_HOLD_MS;
  }
  else if (rightBox && !leftBox) {

    boxBias = 1.0f;                        // positive = right
    boxBiasUntil = nowMs + BOX_HOLD_MS;
  }
  else if (leftBox && rightBox) {

    // Boxes on both sides cancel out
    boxBias = 0.0f;
    boxBiasUntil = 0;
  }

  // ==========================================================
  // THICK BOX  (all three middle sensors dark)
  // ==========================================================

  if (turning) {

    // Still inside a turn: keep pivoting until the timer expires
    if ((int32_t)(nowMs - turnUntil) < 0) {

      pivot(turnDir);
      return;
    }

    // Turn finished
    turning = false;
    turnRearmAt = nowMs + TURN_REARM_MS;

    // Drop the stale PD state so the robot does not lurch
    lastError = 0.0f;
    dFilt = 0.0f;
  }
  else if (
    midCount >= 3 &&
    (int32_t)(nowMs - turnRearmAt) >= 0
  ) {

    // Turn toward the box we just passed, otherwise keep turning
    // the way the line was already curving
    int dir;

    if (boxBias != 0.0f) {
      dir = (boxBias < 0.0f) ? -1 : 1;
    }
    else if (lastSide != 0) {
      dir = lastSide;
    }
    else {
      dir = 1;                             // default: turn right
    }

    turnDir = dir;
    turnUntil = nowMs + (uint32_t)turnMs;
    turning = true;

    pivot(dir);
    return;
  }

  // ==========================================================
  // LINE LOST
  // ==========================================================

  if (midCount == 0) {

    // Start lost timer
    if (lostSince == 0) {
      lostSince = millis();
    }

    // Stop if lost for too long
    if ((millis() - lostSince) > LOST_STOP_MS) {

      stopMotors();

      running = false;

      Serial.println("LINE LOST - STOPPED");

      return;
    }

    // --------------------------------------------------------
    // Search toward last known line position
    // --------------------------------------------------------

    if (lastSide > 0) {

      // Last saw line on RIGHT
      // Spin RIGHT
      setMotors(
        pivotSpeed,
        -pivotSpeed
      );

    }
    else if (lastSide < 0) {

      // Last saw line on LEFT
      // Spin LEFT
      setMotors(
        -pivotSpeed,
        pivotSpeed
      );

    }
    else {

      // No previous direction
      // Default: spin right
      setMotors(
        pivotSpeed,
        -pivotSpeed
      );
    }

    return;
  }

  // ==========================================================
  // LINE FOUND
  // ==========================================================

  lostSince = 0;

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

  // Derivative term
  float derivative = error - lastError;

  // Low-pass filter derivative
  dFilt += 0.35f * (derivative - dFilt);

  // Save current error
  lastError = error;

  // Calculate steering correction
  float correction =
    (Kp * proportional) +
    (Kd * dFilt);

  // Add the box steering kick, fading out over BOX_HOLD_MS
  if (boxBias != 0.0f && (int32_t)(boxBiasUntil - nowMs) > 0) {

    float weight =
      (float)(boxBiasUntil - nowMs) / (float)BOX_HOLD_MS;

    correction += boxBias * boxKick * weight;
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