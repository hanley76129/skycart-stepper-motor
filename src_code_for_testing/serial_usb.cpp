#include "Arduino.h"
#include "USB.h"

#define  SerialCDC  USBSerial

void setup() {

  pinMode(2, OUTPUT);
  SerialCDC.begin(115200);
  SerialCDC.println("Hello from SerialCDC");

}

void loop() {
  delay(1000);
  SerialCDC.println("Hello from SerialCDC111");

}