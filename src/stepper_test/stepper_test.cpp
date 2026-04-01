// ============================================================
//  stepper_test.cpp
//  Skycart Delivery Rack — Stepper Motor Test Firmware
//
//  Characterizes leadscrew lift performance under a 42 lb load.
//  Supports manual speed/direction control over Serial for
//  logging positional accuracy and validating safe operating limits.
//
//  Validated results:
//    12 V max safe speed : 5,000 Hz  (~0.49 in/s)
//    24 V max safe speed : 10,000 Hz (~0.98 in/s)
// ============================================================

#include <Arduino.h>
#include <FastAccelStepper.h>
#include "stepper_test.h"
#include "../shared/motion_config.h"

// ── Module-private globals ────────────────────────────────────
static FastAccelStepperEngine engine;
static FastAccelStepper*      stepper = nullptr;
static bool movingForward  = false;
static bool movingBackward = false;

// ── Helpers ───────────────────────────────────────────────────
static long inchesToSteps(float inches) {
  return lround(inches * STEPS_PER_INCH);
}

static void printStatus() {
  if (!stepper) return;
  long  pos   = stepper->getCurrentPosition();
  float posIn = (float)pos / STEPS_PER_INCH;
  Serial.printf("[STATUS] pos = %ld steps (%.4f in) | running = %s\n",
                pos, posIn, stepper->isRunning() ? "yes" : "no");
}

// ── Command handler ───────────────────────────────────────────
static void handleCommand(char cmd) {
  if (!stepper) return;

  switch (cmd) {
    case '1':  // Run forward
      if (movingBackward) {
        stepper->forceStop();
        movingBackward = false;
        Serial.println("Stopped — reversing to forward.");
      }
      if (!stepper->isRunning()) {
        stepper->runForward();
        movingForward = true;
        Serial.println("Running forward...");
      }
      break;

    case '2':  // Run backward
      if (movingForward) {
        stepper->forceStop();
        movingForward = false;
        Serial.println("Stopped — reversing to backward.");
      }
      if (!stepper->isRunning()) {
        stepper->runBackward();
        movingBackward = true;
        Serial.println("Running backward...");
      }
      break;

    case '3':  // Stop
      stepper->forceStop();
      movingForward  = false;
      movingBackward = false;
      Serial.println("Motor stopped.");
      printStatus();
      break;

    case '4':  // Enable
      stepper->enableOutputs();
      Serial.println("Motor enabled.");
      break;

    case '5':  // Disable — prevents thermal runaway at rest
      stepper->disableOutputs();
      movingForward  = false;
      movingBackward = false;
      Serial.println("Motor disabled (coils de-energized).");
      break;

    case '6':  // +1 inch — used to verify positional accuracy
      if (!stepper->isRunning()) {
        stepper->move(inchesToSteps(1.0f));
        Serial.println("Moving +1.000 inch...");
      }
      break;

    case '7':  // -1 inch
      if (!stepper->isRunning()) {
        stepper->move(inchesToSteps(-1.0f));
        Serial.println("Moving -1.000 inch...");
      }
      break;

    case '8':  // Zero position reference
      stepper->setCurrentPosition(0);
      Serial.println("Position zeroed.");
      break;

    case '9':  // Print status
      printStatus();
      break;

    default:
      Serial.println("Commands: 1=Fwd 2=Bwd 3=Stop 4=Enable 5=Disable"
                     " 6=+1in 7=-1in 8=Zero 9=Status");
      break;
  }

  if (!stepper->isRunning()) {
    movingForward  = false;
    movingBackward = false;
  }
}

// ── Public setup / loop ───────────────────────────────────────
void stepperTestSetup() {
  Serial.println("\n=== Skycart Stepper Motor Test Firmware ===");

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);

  if (!stepper) {
    Serial.println("ERROR: Failed to initialize stepper. Check STEP_PIN.");
    return;
  }

  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(ENABLE_PIN, false);
  stepper->setDelayToEnable(50);
  stepper->setDelayToDisable(50);
  stepper->setAutoEnable(true);
  stepper->setSpeedInHz(TEST_SPEED_HZ);
  stepper->setAcceleration(TEST_ACCEL);

  Serial.printf("Speed    : %u Hz (%.4f in/s)\n",
                TEST_SPEED_HZ, (float)TEST_SPEED_HZ / STEPS_PER_INCH);
  Serial.printf("Accel    : %u Hz/s\n", TEST_ACCEL);
  Serial.printf("Steps/in : %.1f  (1/8 microstepping)\n", STEPS_PER_INCH);
  Serial.println("Commands: 1=Fwd 2=Bwd 3=Stop 4=Enable 5=Disable"
                 " 6=+1in 7=-1in 8=Zero 9=Status");
}

void stepperTestLoop() {
  if (Serial.available() > 0) {
    handleCommand(Serial.read());
  }
}
