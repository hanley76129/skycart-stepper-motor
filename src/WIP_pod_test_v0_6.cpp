#include <Arduino.h>
#include <ESP32_Servo.h>
#include <FastAccelStepper.h>
#include <HardwareSerial.h>


#define STEP_PIN 26
#define DIR_PIN 25
#define ENABLE_PIN 27

FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;

// Stepper move targets
long forwardTarget = 10000;
long backwardTarget = -10000;

// State variables
bool isMovingForward = false;
bool isMovingBackward = false;

void handleCommand(char cmd) {
  if (!stepper)
    return;

  switch (cmd) {
  case 49: // Move forward
    if (isMovingBackward) {
      // Force stop before reversing direction
      stepper->stopMove();
      isMovingBackward = false;
      Serial.println("Motor stopped to reverse direction.");
    }

    if (!stepper->isRunning()) {
      stepper->runForward();
      isMovingForward = true;
      Serial.println("Moving forward...");
    }
    break;

  case 50: // Move backward
    if (isMovingForward) {
      // Force stop before reversing direction
      stepper->stopMove();
      isMovingForward = false;
      Serial.println("Motor stopped to reverse direction.");
    }

    if (!stepper->isRunning()) {
      stepper->runBackward();
      isMovingBackward = true;
      Serial.println("Moving backward...");
    }
    break;

  case 51: // Stop stepper safely
    if (stepper->isRunning()) {
      stepper->stopMove();
      isMovingForward = false;
      isMovingBackward = false;
      Serial.println("Stepper stopped.");
    }
    break;

  case 52:                    // Enable motor
    stepper->enableOutputs(); //
    Serial.println("Stepper enabled.");
    break;

  case 53:                     // Disable motor
    stepper->disableOutputs(); // Turn OFF (release coils, avoid heat)
    isMovingForward = false;
    isMovingBackward = false;
    Serial.println("Stepper disabled.");
    break;

  default:
    Serial.println(
        "Invalid command. Use 1:Forward 2:Backward 3:Stop 4:Enable 5:Disable");
    break;
  }

  // Reset states if move finished
  if (stepper && !stepper->isRunning()) {
    isMovingForward = false;
    isMovingBackward = false;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Stepper Control Setup...");

  // pinMode(STEP_PIN, OUTPUT);
  // pinMode(DIR_PIN, OUTPUT);
  // pinMode(ENABLE_PIN, OUTPUT);

  // digitalWrite(ENABLE_PIN, LOW); // enable motor at start

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);

  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(ENABLE_PIN, false);

    stepper->setDelayToEnable(50);
    stepper->setDelayToDisable(50);

    stepper->setSpeedInUs(1000);
    stepper->setAcceleration(1000);

    Serial.println("Stepper initialized successfully.");
  } else {
    Serial.println("Failed to initialize stepper.");
  }
}

void loop() {
  // Non-blocking serial input
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    handleCommand(cmd);
  }

  // Keep stepper running if there’s a move in progress
  if (stepper && stepper->isRunning()) {
    stepper->keepRunning();
  }
}