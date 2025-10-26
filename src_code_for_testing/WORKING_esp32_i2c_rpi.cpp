// https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/examples/WireSlave/WireSlave.ino

#include "Wire.h"
#include <HardwareSerial.h>
#include <iostream>

#define I2C_DEV_ADDR 0x07

uint32_t i = 0;

void onRequest() {
  Wire.print(i++);
  Wire.print(" Packets.");
  Serial.println("onRequest");
}

// Reference: https://github.com/JohnnySheppard/Pi2c/tree/master
void onReceive(int len) 
{
  int receive_int=0;
  int count=0;
  char byteArray;
  const char *p;
  std::string str_from_rpi;

  Serial.printf("onReceive[%d]: ", len);
  while (Wire.available()) {
    // Original line:
    // Serial.write(Wire.read());
    byteArray = Wire.read();
    Serial.println(byteArray);
    str_from_rpi += byteArray;
    count++;
  }

  std::cout << str_from_rpi << std::endl;

}

void setup() {
  Serial.begin(115200);
  Serial.println("starting");
  Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);

#if CONFIG_IDF_TARGET_ESP32
  char message[64];
  snprintf(message, 64, "%lu Packets.", i++);
  Wire.slaveWrite((uint8_t *)message, strlen(message));
#endif
}

void loop() {}