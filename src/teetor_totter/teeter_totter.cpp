// ============================================================
//  teeter_totter.cpp
//  Skycart Delivery Rack — Teeter-Totter Balance Firmware
//
//  Core idea:
//    Box masses are known before flight. Using the CG formula,
//    we compute exactly where each carriage must sit so the
//    rack's loaded CG lands on the drone's support point.
//    Motors move once to that position — no sensor needed.
//
//  CG formula (rack + boxes):
//    xcg = (Σ mᵢxᵢ + mᵣxᵣ) / (Σmᵢ + mᵣ)
//
//  Layout (top-down, inches, origin = left edge of rack):
//
//    0                      12.5                     25
//    |——[L carriage]——————————|————————[R carriage]——|
//          boxes 0,1      support pt      boxes 2,3
//
//  Loading rule (operator, not firmware):
//    Heaviest box must be placed nearest center before flight.
// ============================================================

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <cmath>
#include "teeter_totter.h"
#include "../shared/motion_config.h"

// ── Module-private constants ──────────────────────────────────
static constexpr int NUM_BOXES = 4;

// ── Module-private globals ────────────────────────────────────
static FastAccelStepperEngine engine;
static FastAccelStepper* stepperL = nullptr;
static FastAccelStepper* stepperR = nullptr;

static float boxMass[NUM_BOXES]      = {0.0f, 0.0f, 0.0f, 0.0f};
static bool  boxDelivered[NUM_BOXES] = {false, false, false, false};
static float carriageL_in = LEFT_HOME_IN;
static float carriageR_in = RIGHT_HOME_IN;

// ── Helpers ───────────────────────────────────────────────────
static long inchesToSteps(float inches) {
  return lround(inches * STEPS_PER_INCH);
}

static float clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ── CG calculation ────────────────────────────────────────────
// Both boxes on a carriage share the carriage x-position (simplified).
// Skips delivered boxes.
static float computeCG(float leftPos, float rightPos) {
  float sumMX = RACK_MASS_KG * RACK_CG_IN;
  float sumM  = RACK_MASS_KG;

  for (int i = 0; i < 2; i++) {
    if (!boxDelivered[i]) { sumMX += boxMass[i] * leftPos;  sumM += boxMass[i]; }
  }
  for (int i = 2; i < NUM_BOXES; i++) {
    if (!boxDelivered[i]) { sumMX += boxMass[i] * rightPos; sumM += boxMass[i]; }
  }

  if (sumM < 1e-6f) return SUPPORT_POINT_IN;
  return sumMX / sumM;
}

// ── Feedforward solver ────────────────────────────────────────
// Solves for carriage positions that place xcg on SUPPORT_POINT_IN.
// Constraint: keep current carriage separation fixed (both slide as a unit).
//
// Derivation:
//   xcg*mT = mL*xL + mR*xR + mRack*xRack
//   xR = xL + sep
//   => xL = (xcg*mT - mRack*xRack - mR*sep) / (mL + mR)
struct CarriageTargets { float left; float right; };

static CarriageTargets solveCarriagePositions() {
  float mL = 0.0f, mR = 0.0f;
  for (int i = 0; i < 2; i++)         if (!boxDelivered[i]) mL += boxMass[i];
  for (int i = 2; i < NUM_BOXES; i++) if (!boxDelivered[i]) mR += boxMass[i];

  CarriageTargets t = {carriageL_in, carriageR_in};

  if ((mL + mR) < 1e-6f) {
    Serial.println("All boxes delivered — carriages stay in place.");
    return t;
  }

  float mT  = mL + mR + RACK_MASS_KG;
  float sep = carriageR_in - carriageL_in;
  float RHS = SUPPORT_POINT_IN * mT - RACK_MASS_KG * RACK_CG_IN - mR * sep;

  t.left  = RHS / (mL + mR);
  t.right = t.left + sep;
  t.left  = clamp(t.left,  LEFT_CARRIAGE_MIN,  LEFT_CARRIAGE_MAX);
  t.right = clamp(t.right, RIGHT_CARRIAGE_MIN, RIGHT_CARRIAGE_MAX);

  float achievedCG = computeCG(t.left, t.right);
  Serial.printf("[SOLVER] Left=%.3f in  Right=%.3f in  →  CG=%.3f in  (target=%.3f in)\n",
                t.left, t.right, achievedCG, SUPPORT_POINT_IN);

  if (fabsf(achievedCG - SUPPORT_POINT_IN) > 0.5f) {
    Serial.println("WARNING: Full CG correction not achievable — carriage at travel limit.");
    Serial.println("         Adjust box placement or update SUPPORT_POINT_IN.");
  }

  return t;
}

// ── Motor commands ────────────────────────────────────────────
static void moveCarriageTo(FastAccelStepper* motor, float targetIn, const char* label) {
  if (!motor) return;
  motor->moveTo(inchesToSteps(targetIn));
  Serial.printf("[MOVE] %s → %.3f in\n", label, targetIn);
}

static void rebalance() {
  Serial.println("\n── Rebalancing ──────────────────────────────────────────");
  Serial.printf("CG before : %.3f in  (error = %+.3f in)\n",
                computeCG(carriageL_in, carriageR_in),
                computeCG(carriageL_in, carriageR_in) - SUPPORT_POINT_IN);

  CarriageTargets targets = solveCarriagePositions();
  moveCarriageTo(stepperL, targets.left,  "LEFT ");
  moveCarriageTo(stepperR, targets.right, "RIGHT");

  unsigned long start = millis();
  while (stepperL->isRunning() || stepperR->isRunning()) {
    delay(10);
    if (millis() - start > 5000UL) {
      Serial.println("WARNING: Rebalance timeout — forcing stop.");
      stepperL->forceStop();
      stepperR->forceStop();
      break;
    }
  }

  carriageL_in = targets.left;
  carriageR_in = targets.right;

  Serial.printf("CG after  : %.3f in  (error = %+.3f in)\n",
                computeCG(carriageL_in, carriageR_in),
                computeCG(carriageL_in, carriageR_in) - SUPPORT_POINT_IN);
  Serial.printf("Done in %lu ms.\n", millis() - start);
  Serial.println("─────────────────────────────────────────────────────────\n");
}

// ── Serial UI ─────────────────────────────────────────────────
static void promptForMasses() {
  Serial.println("\n=== Enter box masses (kg). Press Enter after each. ===");
  Serial.println("    REMINDER: Load heaviest box nearest center before flight.");
  for (int i = 0; i < NUM_BOXES; i++) {
    boxDelivered[i] = false;
    Serial.printf("Box %d mass (kg): ", i);
    while (Serial.available() == 0) delay(10);
    boxMass[i] = Serial.parseFloat();
    while (Serial.available() && Serial.peek() == '\n') Serial.read();
    Serial.printf("  → %.3f kg\n", boxMass[i]);
  }
  Serial.println("Masses recorded.");
}

static void printStatus() {
  Serial.println("\n── Status ───────────────────────────────────────────────");
  for (int i = 0; i < NUM_BOXES; i++) {
    Serial.printf("  Box %d : %.3f kg  %s\n", i, boxMass[i],
                  boxDelivered[i] ? "[DELIVERED]" : "[on rack]");
  }
  Serial.printf("  Left  carriage : %.3f in\n", carriageL_in);
  Serial.printf("  Right carriage : %.3f in\n", carriageR_in);
  Serial.printf("  Current CG     : %.3f in  (target = %.3f in)\n",
                computeCG(carriageL_in, carriageR_in), SUPPORT_POINT_IN);
  Serial.println("─────────────────────────────────────────────────────────\n");
}

static void printHelp() {
  Serial.println("\nCommands:");
  Serial.println("  r       — rebalance now");
  Serial.println("  d<0-3>  — mark box as delivered, rebalance (e.g. 'd2')");
  Serial.println("  m       — re-enter all box masses");
  Serial.println("  s       — emergency stop");
  Serial.println("  p       — print status");
  Serial.println("  ?       — this help\n");
}

// ── Public setup / loop ───────────────────────────────────────
void teeterTotterSetup() {
  Serial.println("\n=== Skycart Teeter-Totter Balance Firmware ===");

  engine.init();
  stepperL = engine.stepperConnectToPin(STEP_PIN_L);
  stepperR = engine.stepperConnectToPin(STEP_PIN_R);

  auto initMotor = [](FastAccelStepper* s, int dirPin, int enPin,
                      float homeIn, const char* label) {
    if (!s) { Serial.printf("ERROR: Failed to init %s stepper.\n", label); return; }
    s->setDirectionPin(dirPin);
    s->setEnablePin(enPin, false);
    s->setDelayToEnable(50);
    s->setDelayToDisable(50);
    s->setAutoEnable(true);
    s->setSpeedInHz(BALANCE_SPEED_HZ);
    s->setAcceleration(BALANCE_ACCEL);
    s->setCurrentPosition(inchesToSteps(homeIn));
    Serial.printf("%s stepper initialized (home = %.1f in).\n", label, homeIn);
  };

  initMotor(stepperL, DIR_PIN_L, ENABLE_PIN_L, LEFT_HOME_IN,  "LEFT ");
  initMotor(stepperR, DIR_PIN_R, ENABLE_PIN_R, RIGHT_HOME_IN, "RIGHT");

  promptForMasses();
  printHelp();
  rebalance();
}

void teeterTotterLoop() {
  if (Serial.available() == 0) return;

  char cmd = Serial.read();

  switch (cmd) {
    case 'r': case 'R': rebalance(); break;

    case 'd': case 'D': {
      unsigned long t = millis();
      while (Serial.available() == 0 && millis() - t < 2000) delay(10);
      if (Serial.available() == 0) { Serial.println("No box index received."); break; }
      int idx = Serial.read() - '0';
      if (idx < 0 || idx >= NUM_BOXES) {
        Serial.printf("Invalid box index. Use 0–%d.\n", NUM_BOXES - 1);
        break;
      }
      if (boxDelivered[idx]) { Serial.printf("Box %d already delivered.\n", idx); break; }
      boxDelivered[idx] = true;
      Serial.printf("Box %d delivered (%.3f kg removed).\n", idx, boxMass[idx]);
      rebalance();
      break;
    }

    case 'm': case 'M':
      stepperL->forceStop();
      stepperR->forceStop();
      promptForMasses();
      rebalance();
      break;

    case 's': case 'S':
      stepperL->forceStop();
      stepperR->forceStop();
      Serial.println("EMERGENCY STOP.");
      break;

    case 'p': case 'P': printStatus(); break;
    case '?':           printHelp();   break;
    default:            break;
  }
}
