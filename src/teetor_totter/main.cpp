// Entry point for the teeter_totter environment.
// All logic is in teeter_totter.cpp — this file just wires it into Arduino.
// To flash: select "teeter_totter" in the PlatformIO toolbar, then upload.

#include <Arduino.h>
#include "teeter_totter/teeter_totter.h"

void setup() {
  Serial.begin(115200);
  teeterTotterSetup();
}

void loop() {
  teeterTotterLoop();
}
