/// Author: jramayrat

/*
  References:
    https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/examples/WireSlave/WireSlave.ino
    https://github.com/JohnnySheppard/Pi2c/tree/master
    https://github.com/espressif/esp-idf/blob/v5.3/examples/system/light_sleep/main/light_sleep_example_main.c

*/

#include <AccelStepper.h>
#include <HardwareSerial.h>
#include <ESP32_Servo.hpp>
#include "Wire.h"

/// @note: These are from the light sleep example. We may not need all these libraries:
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "light_sleep_example.h"
/////////////////////////////////////////////////////////////

#define I2C_DEV_ADDR 0x07

uint32_t i = 0;

/// As of 6-17-2024, this library isn't working so I resorted to simply calling
/// digitalWrites to the stepper-driver-esp32 pins.
AccelStepper front_stepper(AccelStepper::DRIVER, 26, 25); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

// Pins for Interrupt Service Routines
#define INT0_PIN 13
#define INT1_PIN 14

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
#define front_linear_actuator_sig_pin   17
#define tail_linear_actuator_sig_pin    19

/// Front stepper motor pins
#define front_stepper_dir_pin    25
#define front_stepper_step_pin   26
#define front_stepper_enable_pin 27

/// Tail stepper motor pins
#define tail_stepper_dir_pin     21
#define tail_stepper_step_pin    22
#define tail_stepper_enable_pin  23


// Up to 16 servo objects can be created on the ESP32 with the ESP32Servo library
Servo front_linear_actuator;  // create servo object to control a servo
Servo tail_linear_actuator;


/// @brief Global variables for actuator ISRs.
float front_stepper_travel_dist         = 0;
float tail_stepper_travel_dist          = 0;
float front_linear_actuator_travel_dist = 0;
float tail_linear_actuator_travel_dist  = 0;


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

  Serial.printf("onReceive[%d]: ", len);
  while (Wire.available()) {
    // Original line:
    // Serial.write(Wire.read());
    char c = Wire.read();
    receive_int = c << (8 * count) | receive_int;
    count++;
  }
  //Print the Int out.
  Serial.print("Received Number: "); 
  Serial.println(receive_int);
  
  // Serial.println();
}


void IRAM_ATTR interrupt_func1(void)
{

	front_linear_actuator.write(360);    // tell servo to go to position in variable 'pos'
	tail_linear_actuator.write(360);
	delay(40000);             // waits 15ms for the servo to reach the position
	// Serial.println(pos);

	// int pos = 0;    // variable to store the servo position

	// for (pos = 0; pos <= 400; pos += 1) 
	// { // goes from 0 degrees to 180 degrees
	// 	// in steps of 1 degree
	// 	short_side_linear_actuator.write(0);    // tell servo to go to position in variable 'pos'
	// 	delay(100);             // waits 150ms for the servo to reach the position
	// 	// Serial.println(pos);
	// }

}


static void light_sleep_task(void *args)
{
    while (true) {
        printf("Entering light sleep\n");
        /* To make sure the complete line is printed before entering sleep mode,
         * need to wait until UART TX FIFO is empty:
         */
        uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);

        /* Get timestamp before entering sleep */
        int64_t t_before_us = esp_timer_get_time();

        /* Enter sleep mode */
        esp_light_sleep_start();

        /* Get timestamp after waking up from sleep */
        int64_t t_after_us = esp_timer_get_time();

        /* Determine wake up reason */
        const char* wakeup_reason;
        switch (esp_sleep_get_wakeup_cause()) {
            case ESP_SLEEP_WAKEUP_TIMER:
                wakeup_reason = "timer";
                break;
            case ESP_SLEEP_WAKEUP_GPIO:
                wakeup_reason = "pin";
                break;
            case ESP_SLEEP_WAKEUP_UART:
                wakeup_reason = "uart";
                /* Hang-up for a while to switch and execute the uart task
                 * Otherwise the chip may fall sleep again before running uart task */
                vTaskDelay(1);
                break;
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
            case ESP_SLEEP_WAKEUP_TOUCHPAD:
                wakeup_reason = "touch";
                break;
#endif
            default:
                wakeup_reason = "other";
                break;
        }
#if CONFIG_NEWLIB_NANO_FORMAT
        /* printf in newlib-nano does not support %ll format, causing example test fail */
        printf("Returned from light sleep, reason: %s, t=%d ms, slept for %d ms\n",
                wakeup_reason, (int) (t_after_us / 1000), (int) ((t_after_us - t_before_us) / 1000));
#else
        printf("Returned from light sleep, reason: %s, t=%lld ms, slept for %lld ms\n",
                wakeup_reason, t_after_us / 1000, (t_after_us - t_before_us) / 1000);
#endif
        if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
            /* Waiting for the gpio inactive, or the chip will continuously trigger wakeup*/
            example_wait_gpio_inactive();
        }
    }
    vTaskDelete(NULL);
}


void setup()
{  
  Serial.begin(115200);
  esp_light_sleep_start();  

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

  digitalWrite(front_stepper_step_pin, HIGH);

  front_stepper.setEnablePin(27);
  front_stepper.setPinsInverted(false, false, true);
  front_stepper.setMinPulseWidth(20);
  front_stepper.disableOutputs();
  front_stepper.setMaxSpeed(1000);
  front_stepper.setSpeed(50);	


	Serial.println("Starting..");
	// Allow allocation of all timers
	// ESP32PWM::allocateTimer(0);
	// ESP32PWM::allocateTimer(1);
	// ESP32PWM::allocateTimer(2);
	// ESP32PWM::allocateTimer(3);
	// short_side_linear_actuator.setPeriodHertz(50);    // standard 50 hz servo
	front_linear_actuator.attach(front_linear_actuator_sig_pin, 1000, 2000); // attaches the servo on pin 18 to the servo object
	tail_linear_actuator.attach(tail_linear_actuator_sig_pin, 1000, 2000); // attaches the servo on pin 18 to the servo object

	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep

  pinMode(INT0_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(INT0_PIN), interrupt_func1, RISING);
  pinMode(INT1_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(INT1_PIN), interrupt_func2, RISING);

  rtc_gpio_pulldown_en();

}

void loop()
{  
  Serial.println("loop");

  // front_stepper.runSpeed();

  // front_stepper.run();
}
