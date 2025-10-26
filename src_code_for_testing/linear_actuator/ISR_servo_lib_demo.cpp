#ifndef ESP32
  #error This code is designed to run on ESP32 platform, not Arduino nor ESP8266! Please check your Tools->Board setting.
#endif

#define TIMER_INTERRUPT_DEBUG       1
#define ISR_SERVO_DEBUG             1

// Select different ESP32 timer number (0-3) to avoid conflict
#define USE_ESP32_TIMER_NO          3

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "D:\Projects\Skycart\nimbus_software\Multi-payload\sandbox-esp-code\lib\ESP32_ISR_Servo\src\ESP32_ISR_Servo.hpp"
#include "D:\Projects\Skycart\nimbus_software\Multi-payload\sandbox-esp-code\lib\ESP32_ISR_Servo\src\ESP32_ISR_Servo_Impl.hpp"
// #include "ESP32_New_ISR_Servo.h"

//See file .../hardware/espressif/esp32/variants/(esp32|doitESP32devkitV1)/pins_arduino.h
#if !defined(LED_BUILTIN)
  #define LED_BUILTIN       2         // Pin D2 mapped to pin GPIO2/ADC12 of ESP32, control on-board LED
#endif

#define PIN_LED           2         // Pin D2 mapped to pin GPIO2/ADC12 of ESP32, control on-board LED

#define PIN_D0            0         // Pin D0 mapped to pin GPIO0/BOOT/ADC11/TOUCH1 of ESP32
#define PIN_D1            1         // Pin D1 mapped to pin GPIO1/TX0 of ESP32
#define PIN_D2            2         // Pin D2 mapped to pin GPIO2/ADC12/TOUCH2 of ESP32
#define PIN_D3            3         // Pin D3 mapped to pin GPIO3/RX0 of ESP32
#define PIN_D4            4         // Pin D4 mapped to pin GPIO4/ADC10/TOUCH0 of ESP32
#define PIN_D5            5         // Pin D5 mapped to pin GPIO5/SPISS/VSPI_SS of ESP32
#define PIN_D6            6         // Pin D6 mapped to pin GPIO6/FLASH_SCK of ESP32
#define PIN_D7            7         // Pin D7 mapped to pin GPIO7/FLASH_D0 of ESP32
#define PIN_D8            8         // Pin D8 mapped to pin GPIO8/FLASH_D1 of ESP32
#define PIN_D9            9         // Pin D9 mapped to pin GPIO9/FLASH_D2 of ESP32

#define PIN_D19           19
// Published values for SG90 servos; adjust if needed
// #define MIN_MICROS      800  //544
// #define MAX_MICROS      2450

#define MIN_MICROS      0  //544
#define MAX_MICROS      360


int servoIndex1  = 0;

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  delay(200);

  Serial.print(F("\nStarting ISR Servo on ")); Serial.println(ARDUINO_BOARD);
  Serial.println(ESP32_ISR_SERVO_VERSION);
  
  //Select ESP32 timer USE_ESP32_TIMER_NO
  ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);
  // ESP32_ISR_Servos.setPosition(180);
  servoIndex1 = ESP32_ISR_Servos.setupServo(PIN_D19, MIN_MICROS, MAX_MICROS);

  // if (servoIndex1 != -1)
  //   Serial.println(F("Setup Servo1 OK"));
  // else
  //   Serial.println(F("Setup Servo1 failed"));

  // if (servoIndex2 != -1)
  //   Serial.println(F("Setup Servo2 OK"));
  // else
  //   Serial.println(F("Setup Servo2 failed"));
}

void loop()
{
  Serial.println(F("loop iteration"));
  uint16_t pulse_width = 2000;
  ESP32_ISR_Servos.setPulseWidth(0, pulse_width);

  int position = 180;
  ESP32_ISR_Servos.setPosition(servoIndex1, position);

    // if ( ( servoIndex1 != -1) && ( servoIndex2 != -1) )
    // {
    //   for (position = 0; position <= 180; position++)
    //   {
    //     // goes from 0 degrees to 180 degrees
    //     // in steps of 1 degree

    //     if (position % 30 == 0)
    //     {
    //       Serial.print(F("Servo1 pos = ")); Serial.print(position);
    //       Serial.print(F(", Servo2 pos = ")); Serial.println(180 - position);
    //     }

    //     ESP32_ISR_Servos.setPosition(servoIndex1, position);
    //     ESP32_ISR_Servos.setPosition(servoIndex2, 180 - position);
    //     // waits 30ms for the servo to reach the position
    //     delay(30);
    //   }
      
    //   delay(5000);

    //   for (position = 180; position >= 0; position--)
    //   {
    //     // goes from 180 degrees to 0 degrees
    //     if (position % 30 == 0)
    //     {
    //       Serial.print(F("Servo1 pos = ")); Serial.print(position);
    //       Serial.print(F(", Servo2 pos = ")); Serial.println(180 - position);
    //     }

    //     ESP32_ISR_Servos.setPosition(servoIndex1, position);
    //     ESP32_ISR_Servos.setPosition(servoIndex2, 180 - position);
    //     // waits 30ms for the servo to reach the position
    //     delay(30);
    //   }
      
    //   delay(5000);
    // }
}