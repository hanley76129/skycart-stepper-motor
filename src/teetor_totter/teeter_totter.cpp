// ============================================================
//  teeter_totter.cpp
//  Skycart Delivery Rack — Teeter-Totter Balance Firmware
//
//  Physical layout (top-down, inches, origin = left edge):
//
//    [motorL]=[pusherL][box0][box1][box2][pusherR]=[motorR]
//    0"       |  2"   | 6"  | 6"  | 6"  |  2"  |   25"
//             pL                          pR
//
//  pL = inner face of left pusher  (contacts leftmost box)
//  pR = inner face of right pusher (contacts rightmost box)
//
//  3 boxes, each 6" wide along rack axis. Pushers 2" wide.
//  Boxes slide freely on the rack surface.
//  Drop hole centered at 12.5" with a two-servo trapdoor.
//
//  Compressed stack invariant:
//    Remaining boxes are always packed together with both
//    pushers in contact. No gaps. Box positions are fully
//    determined by pL and the box stacking order.
//
//    Box CG   = pL + (rank + 0.5) * BOX_WIDTH
//    pR       = pL + N * BOX_WIDTH    (N = live box count)
//
//  Delivery model:
//    The drone lands before each delivery. CG balance is only
//    required in flight. The delivery sequence is:
//      1. Land.
//      2. Slide stack to position target box over hole.
//      3. Open trapdoor servos. Box falls through.
//      4. Close trapdoor servos.
//      5. Compress remaining boxes, rebalance CG for flight.
//      6. Take off.
//
//  Delivery order:
//    Box 1 (center, heaviest) is delivered first since it
//    starts over the hole. Remaining boxes are delivered in
//    an order specified by the operator.
//
//  CG formula:
//    xcg = (sum(mi * xi) + mRack * xRack) / (sum(mi) + mRack)
// ============================================================

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <ESP32Servo.h>
#include <cmath>
#include "teeter_totter.h"
#include "../shared/motion_config.h"

// ── Module-private globals ────────────────────────────────────
static FastAccelStepperEngine engine;
static FastAccelStepper* stepperL = nullptr;
static FastAccelStepper* stepperR = nullptr;

static Servo servoL;
static Servo servoR;

static float boxMass[NUM_BOXES]      = {};
static bool  boxDelivered[NUM_BOXES] = {};
static float pusherL = PUSHER_L_HOME;
static float pusherR = PUSHER_R_HOME;

// ── Helpers ───────────────────────────────────────────────────
static long inchesToSteps(float inches) {
  return lround(inches * STEPS_PER_INCH);
}

static float clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ── Trapdoor control ──────────────────────────────────────────
static void trapdoorOpen() {
  servoL.write(SERVO_OPEN_DEG);
  servoR.write(SERVO_OPEN_DEG);
  Serial.println("[TRAPDOOR] Open.");
}

static void trapdoorClose() {
  servoL.write(SERVO_CLOSED_DEG);
  servoR.write(SERVO_CLOSED_DEG);
  Serial.println("[TRAPDOOR] Closed.");
}

// ── Box geometry (compressed stack) ───────────────────────────
// Rank: position among remaining boxes, counted from the left.
static int boxRank(int idx) {
  int rank = 0;
  for (int i = 0; i < idx; i++) {
    if (!boxDelivered[i]) rank++;
  }
  return rank;
}

static int liveBoxCount() {
  int n = 0;
  for (int i = 0; i < NUM_BOXES; i++) {
    if (!boxDelivered[i]) n++;
  }
  return n;
}

static float boxCG(int idx, float pL) {
  if (boxDelivered[idx]) return 0.0f;
  return pL + (boxRank(idx) + 0.5f) * BOX_WIDTH_IN;
}

// ── CG calculation ────────────────────────────────────────────
static float computeCG(float pL) {
  float sumMX = RACK_MASS_KG * RACK_CG_IN;
  float sumM  = RACK_MASS_KG;

  for (int i = 0; i < NUM_BOXES; i++) {
    if (!boxDelivered[i]) {
      sumMX += boxMass[i] * boxCG(i, pL);
      sumM  += boxMass[i];
    }
  }

  if (sumM < 1e-6f) return SUPPORT_POINT_IN;
  return sumMX / sumM;
}

// ── Balance solver ────────────────────────────────────────────
// Solves for pL that places the loaded CG at the support point.
//
//   SP * mT = mBoxes * pL + momentOffset + mRack * xRack
//   pL = (SP * mT - momentOffset - mRack * xRack) / mBoxes
//
// where momentOffset = sum[ mi * (ri + 0.5) * BOX_WIDTH ]
//
struct PusherTargets { float left; float right; };

static PusherTargets solveBalance() {
  int N = liveBoxCount();
  PusherTargets t = {pusherL, pusherR};

  if (N == 0) {
    Serial.println("All boxes delivered.");
    return t;
  }

  float mBoxes = 0.0f;
  float momentOffset = 0.0f;

  for (int i = 0; i < NUM_BOXES; i++) {
    if (!boxDelivered[i]) {
      mBoxes += boxMass[i];
      momentOffset += boxMass[i] * (boxRank(i) + 0.5f) * BOX_WIDTH_IN;
    }
  }

  float mT = mBoxes + RACK_MASS_KG;
  t.left  = (SUPPORT_POINT_IN * mT - momentOffset - RACK_MASS_KG * RACK_CG_IN) / mBoxes;
  t.right = t.left + N * BOX_WIDTH_IN;

  float pLmax = PUSHER_R_MAX - N * BOX_WIDTH_IN;
  t.left  = clamp(t.left, PUSHER_L_MIN, pLmax);
  t.right = t.left + N * BOX_WIDTH_IN;

  float achieved = computeCG(t.left);
  Serial.printf("[BALANCE] pL=%.2f  pR=%.2f  CG=%.3f (target=%.3f, error=%+.3f)\n",
                t.left, t.right, achieved, SUPPORT_POINT_IN,
                achieved - SUPPORT_POINT_IN);

  if (fabsf(achieved - SUPPORT_POINT_IN) > 0.5f) {
    Serial.println("WARNING: CG correction limited by pusher travel.");
  }

  return t;
}

// ── Delivery positioning ──────────────────────────────────────
// Slides the compressed stack so the target box is centered
// over the drop hole. Called while the drone is on the ground.
static bool solveDeliver(int idx, PusherTargets& t) {
  if (boxDelivered[idx]) {
    Serial.printf("Box %d already delivered.\n", idx);
    return false;
  }

  if (idx != 1 && !boxDelivered[1]) {
    Serial.println("Box 1 (center) must be delivered first.");
    return false;
  }

  int N = liveBoxCount();
  int r = boxRank(idx);

  t.left  = HOLE_POS_IN - (r + 0.5f) * BOX_WIDTH_IN;
  t.right = t.left + N * BOX_WIDTH_IN;

  float pLmax = PUSHER_R_MAX - N * BOX_WIDTH_IN;
  t.left  = clamp(t.left, PUSHER_L_MIN, pLmax);
  t.right = t.left + N * BOX_WIDTH_IN;

  float targetCG = boxCG(idx, t.left);
  if (fabsf(targetCG - HOLE_POS_IN) > 0.5f) {
    Serial.printf("WARNING: Box %d can only reach %.2f (hole at %.2f).\n",
                  idx, targetCG, HOLE_POS_IN);
  }

  Serial.printf("[DELIVER] box %d -> hole  pL=%.2f  pR=%.2f\n", idx, t.left, t.right);
  return true;
}

// ── Motor commands ────────────────────────────────────────────
static void movePusherTo(FastAccelStepper* motor, float targetIn, const char* label) {
  if (!motor) return;
  motor->moveTo(inchesToSteps(targetIn));
  Serial.printf("[MOVE] %s -> %.3f in\n", label, targetIn);
}

// Both motors run concurrently via independent hardware timer
// ISRs. Blocks until both reach their targets.
static void executeMove(PusherTargets targets) {
  movePusherTo(stepperL, targets.left,  "LEFT ");
  movePusherTo(stepperR, targets.right, "RIGHT");

  unsigned long start = millis();
  while (stepperL->isRunning() || stepperR->isRunning()) {
    delay(10);
    if (millis() - start > 5000UL) {
      Serial.println("WARNING: Move timeout -- forcing stop.");
      stepperL->forceStop();
      stepperR->forceStop();
      break;
    }
  }

  pusherL = targets.left;
  pusherR = targets.right;
  Serial.printf("Move complete in %lu ms.\n", millis() - start);
}

// Compresses remaining boxes and rebalances CG for flight.
static void rebalance() {
  Serial.println("\n-- Compress + Rebalance --");
  PusherTargets targets = solveBalance();
  executeMove(targets);
  Serial.printf("CG = %.3f in  (ready for flight)\n\n",
                computeCG(pusherL));
}

// Full delivery sequence: open trapdoor, wait, close, mark, rebalance.
static void deliverBox(int idx) {
  if (boxDelivered[idx]) { Serial.printf("Box %d already delivered.\n", idx); return; }

  Serial.printf("\n-- Delivering box %d (%.3f kg) --\n", idx, boxMass[idx]);

  trapdoorOpen();
  delay(DROP_DELAY_MS);
  trapdoorClose();

  boxDelivered[idx] = true;
  Serial.printf("Box %d delivered.\n", idx);
  rebalance();
}

// ── Serial UI ─────────────────────────────────────────────────
static void promptForMasses() {
  Serial.println("\n=== Enter box masses (kg). Press Enter after each. ===");
  Serial.println("    Layout: [box0 left][box1 center][box2 right]");
  Serial.println("    Box 1 (center) should be heaviest.");
  for (int i = 0; i < NUM_BOXES; i++) {
    boxDelivered[i] = false;
    Serial.printf("Box %d mass (kg): ", i);
    while (Serial.available() == 0) delay(10);
    boxMass[i] = Serial.parseFloat();
    while (Serial.available() && Serial.peek() == '\n') Serial.read();
    Serial.printf("  -> %.3f kg\n", boxMass[i]);
  }
  Serial.println("Masses recorded.");
}

static void printStatus() {
  int N = liveBoxCount();
  Serial.println("\n-- Status --");
  for (int i = 0; i < NUM_BOXES; i++) {
    if (boxDelivered[i]) {
      Serial.printf("  Box %d : %.3f kg  [DELIVERED]\n", i, boxMass[i]);
    } else {
      Serial.printf("  Box %d : %.3f kg  @ %.2f in  (rank %d)\n",
                    i, boxMass[i], boxCG(i, pusherL), boxRank(i));
    }
  }
  Serial.printf("  Left  pusher : %.2f in\n", pusherL);
  Serial.printf("  Right pusher : %.2f in\n", pusherR);
  Serial.printf("  Stack        : %.1f in  (%d boxes)\n", N * BOX_WIDTH_IN, N);
  Serial.printf("  CG           : %.3f in  (target %.3f)\n\n",
                computeCG(pusherL), SUPPORT_POINT_IN);
}

static void printHelp() {
  Serial.println("\nCommands:");
  Serial.println("  r       -- compress + rebalance for flight");
  Serial.println("  h<0-2>  -- slide box to drop hole (e.g. 'h2')");
  Serial.println("  d<0-2>  -- open trapdoor, drop box, rebalance (e.g. 'd1')");
  Serial.println("  o       -- open trapdoor (test)");
  Serial.println("  c       -- close trapdoor (test)");
  Serial.println("  m       -- re-enter box masses");
  Serial.println("  s       -- emergency stop");
  Serial.println("  p       -- print status");
  Serial.println("  ?       -- help");
  Serial.println("\nDelivery flow: h1 -> d1 -> h_ -> d_ -> h_ -> d_\n");
}

// ── Public setup / loop ───────────────────────────────────────
void teeterTotterSetup() {
  Serial.println("\n=== Skycart Teeter-Totter Balance Firmware ===");

  // Stepper motors
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
    Serial.printf("%s stepper init (home = %.1f in)\n", label, homeIn);
  };

  initMotor(stepperL, DIR_PIN_L, ENABLE_PIN_L, PUSHER_L_HOME, "LEFT ");
  initMotor(stepperR, DIR_PIN_R, ENABLE_PIN_R, PUSHER_R_HOME, "RIGHT");

  // Trapdoor servos
  servoL.attach(SERVO_L_PIN);
  servoR.attach(SERVO_R_PIN);
  trapdoorClose();
  Serial.println("Trapdoor servos initialized (closed).");

  promptForMasses();
  printHelp();
  rebalance();
}

void teeterTotterLoop() {
  if (Serial.available() == 0) return;

  char cmd = Serial.read();

  switch (cmd) {
    case 'r': case 'R': rebalance(); break;

    case 'h': case 'H': {
      unsigned long t = millis();
      while (Serial.available() == 0 && millis() - t < 2000) delay(10);
      if (Serial.available() == 0) { Serial.println("No box index."); break; }
      int idx = Serial.read() - '0';
      if (idx < 0 || idx >= NUM_BOXES) {
        Serial.printf("Invalid index. Use 0-%d.\n", NUM_BOXES - 1);
        break;
      }
      PusherTargets targets;
      if (solveDeliver(idx, targets)) {
        executeMove(targets);
        Serial.printf("Box %d over hole. Send 'd%d' to drop.\n", idx, idx);
      }
      break;
    }

    case 'd': case 'D': {
      unsigned long t = millis();
      while (Serial.available() == 0 && millis() - t < 2000) delay(10);
      if (Serial.available() == 0) { Serial.println("No box index."); break; }
      int idx = Serial.read() - '0';
      if (idx < 0 || idx >= NUM_BOXES) {
        Serial.printf("Invalid index. Use 0-%d.\n", NUM_BOXES - 1);
        break;
      }
      deliverBox(idx);
      break;
    }

    case 'o': case 'O': trapdoorOpen();  break;
    case 'c': case 'C': trapdoorClose(); break;

    case 'm': case 'M':
      stepperL->forceStop();
      stepperR->forceStop();
      promptForMasses();
      rebalance();
      break;

    case 's': case 'S':
      stepperL->forceStop();
      stepperR->forceStop();
      trapdoorClose();
      Serial.println("EMERGENCY STOP.");
      break;

    case 'p': case 'P': printStatus(); break;
    case '?':           printHelp();   break;
    default:            break;
  }
}
