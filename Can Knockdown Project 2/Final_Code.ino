#include <AccelStepper.h>

// MKS Base V1.6 (RAMPS 1.4 pinout)
#define X_STEP 54
#define X_DIR  55
#define X_EN   38
#define Y_STEP 60
#define Y_DIR  61
#define Y_EN   56
#define Z_STEP 46
#define Z_DIR  48
#define Z_EN   62
#define E0_STEP 26   // second X motor
#define E0_DIR  28
#define E0_EN   24

// Left joystick (1): X/Y.  Right joystick (2): Z.
#define JOY1_X  A15  // VRX
#define JOY1_Y  A14  // VRY
#define JOY2_Z  A13  // joystick 2 up/down

// Left joystick push button (SW pin).
// Wire SW -> "S" pin of the X-MIN endstop header (D3), joystick GND -> GND.
#define JOY1_SW A12

// ---- Calibration ----
// PLACEHOLDER: GT2 belt, 20-tooth pulley = 2032 steps/inch. Measure and correct.
const float XY_STEPS_PER_INCH = 2032.0;
// PLACEHOLDER: if Z uses a lead screw (e.g. T8), this will be very different
// (T8, 8 mm lead, 1/16 microstepping ~ 10160 steps/inch). Measure it.
const float Z_STEPS_PER_INCH  = 2032.0;

// ---- Travel limits (inches from the power-up / zero position) ----
const float X_MAX_IN = 10.5;
const float Y_MAX_IN = 10.2;
const float Z_MAX_IN = 5.0;

const long X_MAX_STEPS = (long)(X_MAX_IN * XY_STEPS_PER_INCH);
const long Y_MAX_STEPS = (long)(Y_MAX_IN * XY_STEPS_PER_INCH);
const long Z_MAX_STEPS = (long)(Z_MAX_IN * Z_STEPS_PER_INCH);

// ---- Tune these ----
const float JOG_SPEED   = 2000.0;   // X/Y steps/sec at full deflection
const float Z_JOG_SPEED = 1000.0;   // Z steps/sec at full deflection
const float HOME_SPEED  = 1000.0;   // steps/sec when returning to zero
const int   DEADZONE    = 80;
const bool  INVERT_X    = false;
const bool  INVERT_Y    = false;
const bool  INVERT_Z    = false;

AccelStepper stepperX (AccelStepper::DRIVER, X_STEP,  X_DIR);
AccelStepper stepperX2(AccelStepper::DRIVER, E0_STEP, E0_DIR);
AccelStepper stepperY (AccelStepper::DRIVER, Y_STEP,  Y_DIR);
AccelStepper stepperZ (AccelStepper::DRIVER, Z_STEP,  Z_DIR);

const int enPins[] = {X_EN, Y_EN, Z_EN, E0_EN};

int centerX, centerY, centerZ;
unsigned long lastRead = 0;

float axisSpeed(int d, bool invert, float maxSpeed) {
  if (abs(d) < DEADZONE) return 0;
  float s = constrain((float)d / 512.0, -1.0, 1.0) * maxSpeed;
  return invert ? -s : s;
}

void setMotorsEnabled(bool on) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(enPins[i], on ? LOW : HIGH);   // LOW = enabled
  }
}

// Step one motor toward position 0 (call repeatedly)
void stepToZero(AccelStepper &m) {
  long p = m.currentPosition();
  if (p == 0) { m.setSpeed(0); return; }
  m.setSpeed(p > 0 ? -HOME_SPEED : HOME_SPEED);
  m.runSpeed();
}

void goHome() {
  while (stepperX.currentPosition()  != 0 || stepperX2.currentPosition() != 0 ||
         stepperY.currentPosition()  != 0 || stepperZ.currentPosition()  != 0) {
    stepToZero(stepperX);
    stepToZero(stepperX2);
    stepToZero(stepperY);
    stepToZero(stepperZ);
  }
}

// Left joystick press: return to zero, cut power to the motors, and stop.
// The machine does nothing else until it is power-cycled (or RESET is pressed),
// and on the next power-up it starts again from the same zero.
void shutdownMachine() {
  goHome();
  setMotorsEnabled(false);
  while (true) {}          // halted: joysticks ignored
}

void setup() {
  for (int i = 0; i < 4; i++) pinMode(enPins[i], OUTPUT);
  setMotorsEnabled(true);

  pinMode(JOY1_SW, INPUT_PULLUP);   // pressed = LOW

  // Keeps the two X motors turning the same way on the carriage
  stepperX2.setPinsInverted(false, false, false);

  stepperX.setMaxSpeed(max(JOG_SPEED, HOME_SPEED));
  stepperX2.setMaxSpeed(max(JOG_SPEED, HOME_SPEED));
  stepperY.setMaxSpeed(max(JOG_SPEED, HOME_SPEED));
  stepperZ.setMaxSpeed(max(Z_JOG_SPEED, HOME_SPEED));

  // Calibrate stick centers (don't touch either stick at power-up)
  long ax = 0, ay = 0, az = 0;
  for (int i = 0; i < 20; i++) {
    ax += analogRead(JOY1_X);
    ay += analogRead(JOY1_Y);
    az += analogRead(JOY2_Z);
    delay(5);
  }
  centerX = ax / 20;
  centerY = ay / 20;
  centerZ = az / 20;
}

void loop() {
  // Left joystick press (debounced) -> go home and shut down
  if (digitalRead(JOY1_SW) == LOW) {
    delay(30);
    if (digitalRead(JOY1_SW) == LOW) shutdownMachine();
  }

  // Read joysticks ~50 times/sec
  if (millis() - lastRead >= 20) {
    lastRead = millis();

    float sx = axisSpeed(analogRead(JOY1_X) - centerX, INVERT_X, JOG_SPEED);
    float sy = axisSpeed(analogRead(JOY1_Y) - centerY, INVERT_Y, JOG_SPEED);
    float sz = axisSpeed(analogRead(JOY2_Z) - centerZ, INVERT_Z, Z_JOG_SPEED);

    stepperX.setSpeed(sx);
    stepperX2.setSpeed(sx);           // X2 always matches X
    stepperY.setSpeed(sy);
    stepperZ.setSpeed(sz);
  }

  // Software limits checked every step so the axes stop exactly at the limit
  long px = stepperX.currentPosition();
  long py = stepperY.currentPosition();
  long pz = stepperZ.currentPosition();
  float vx = stepperX.speed(), vy = stepperY.speed(), vz = stepperZ.speed();

  bool xOk = !((vx > 0 && px >= X_MAX_STEPS) || (vx < 0 && px <= 0));
  bool yOk = !((vy > 0 && py >= Y_MAX_STEPS) || (vy < 0 && py <= 0));
  bool zOk = !((vz > 0 && pz >= Z_MAX_STEPS) || (vz < 0 && pz <= 0));

  if (xOk) { stepperX.runSpeed(); stepperX2.runSpeed(); }
  if (yOk) stepperY.runSpeed();
  if (zOk) stepperZ.runSpeed();
}