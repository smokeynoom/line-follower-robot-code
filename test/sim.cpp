// Host-side check of the box / turn logic.
//   g++ -std=c++17 -Itest/stub -x c++ test/sim.cpp -o /tmp/sim && /tmp/sim
#include "../FastLineFollower.ino"
#include <cassert>

// Which sensors see dark, left to right: IR1..IR5
void see(bool l, bool a, bool b, bool c, bool r) {
  uint32_t dark = (l ? M_IR1 : 0) | (a ? M_IR2 : 0) | (b ? M_IR3 : 0) |
                  (c ? M_IR4 : 0) | (r ? M_IR5 : 0);
  GPIO.in = ~dark;
}
void run(int ms) { for (int i = 0; i < ms * 4; i++) { simUs += 250; loop(); } }
int leftMotor()  { return (pinVal[IN1] ? 1 : -1) * pwmVal[ENA]; }
int rightMotor() { return (pinVal[IN4] ? 1 : -1) * pwmVal[ENB]; }

void start() {
  setup();
  running = true; phase = FOLLOW; pendingTurn = 0; lastSide = 0;
  leftRun = rightRun = 0; thickRun = 0; turnRearmAt = millis();
  see(0,0,1,0,0); run(50);
}

int main() {
  // Left box passed long before the thick box -> turn LEFT
  start();
  see(1,0,1,0,0); run(10);
  see(0,0,1,0,0); run(500);              // far longer than the old 120 ms
  assert(pendingTurn == -1);
  see(1,1,1,1,1); run(5);                // wide thick box covers all five
  assert(phase == CREEP && turnDir == -1 && pendingTurn == 0);
  run(creepMs);
  see(0,0,0,0,0); run(20);
  assert(phase == SPIN && leftMotor() < 0 && rightMotor() > 0);
  see(1,0,0,0,0); run(5);                // spin sweeps over a box: ignored
  see(0,0,1,0,0); run(1);
  assert(phase == FOLLOW);
  run(400);
  assert(pendingTurn == 0);

  // Right box -> turn RIGHT
  start();
  see(0,0,1,0,1); run(10); see(0,0,1,0,0); run(50);
  see(0,1,1,1,0); run(5);
  assert(turnDir == 1);

  // A wide box seen by the outer sensors is NOT a side box
  start();
  see(1,1,1,1,1); run(2); see(0,0,1,0,0); run(5);   // too short to turn
  assert(phase == FOLLOW && pendingTurn == 0);

  // Crossing line (both outer sensors) is not a side box either
  start();
  see(1,0,1,0,1); run(10); see(0,0,1,0,0); run(5);
  assert(pendingTurn == 0);

  // One-sample speck under all three middle sensors: no turn
  start();
  see(0,1,1,1,0); run(1); see(0,0,1,0,0); run(5);
  assert(phase == FOLLOW);

  // Line slips off the right: arc first, spin in place later
  start();
  see(0,0,0,1,0); run(20);
  see(0,0,0,0,0); run(50);
  assert(leftMotor() > 0 && rightMotor() >= 0 && leftMotor() > rightMotor());
  run(200);
  assert(leftMotor() > 0 && rightMotor() < 0);

  // D term lasts tens of ms, not one sample: a step 0 -> +1 steers
  // harder at 10 ms (P + D) than at 50 ms (P only)
  start();
  see(0,0,1,1,0); run(10);
  int early = leftMotor() - rightMotor();
  run(40);
  int late = leftMotor() - rightMotor();
  assert(early > late && late > 0);

  // Straight boost: base 200 ramps up to boost 255
  start();
  baseSpeed = 200; boostSpeed = 255;
  see(0,0,1,0,0); run(20);
  assert(leftMotor() == 200);
  run(500);
  assert(leftMotor() == 255 && rightMotor() == 255);

  // Curve brake: leaving the boosted straight reverses both briefly
  brakeMs = 15;
  see(0,0,1,1,0); run(5);
  assert(leftMotor() < 0 && rightMotor() < 0);
  run(15);
  assert(leftMotor() > rightMotor() && rightMotor() >= 0);
  brakeMs = 0; baseSpeed = 240;

  puts("all box/turn/speed checks passed");
}
