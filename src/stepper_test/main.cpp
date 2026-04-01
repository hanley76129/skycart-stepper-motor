// Entry point for the stepper_test environment.
// All logic is in stepper_test.cpp — this file just wires it into Arduino.
// To flash: select "stepper_test" in the PlatformIO toolbar, then upload.

#include <Arduino.h>
#include "stepper_test/stepper_test.h"

void setup() {
  Serial.begin(115200);
  stepperTestSetup();
}

void loop() {
  stepperTestLoop();
}
