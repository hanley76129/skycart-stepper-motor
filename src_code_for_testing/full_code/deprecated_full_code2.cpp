/// Author: jramayrat

/*
  References:
    https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/examples/WireSlave/WireSlave.ino
    https://github.com/JohnnySheppard/Pi2c/tree/master

// Be cautious of storing variables https://www.w3schools.com/cpp/cpp_data_types.asp

*****************************************************************************************************************
(This section can be deleted when finished reviewing)
Notes for Josh on memory efficient use: 
lets say we have an I2C device register at 0x07, what is this in binary?
uint16 I2C_DEV_REG 0x07 = 0000 0111
in embedded device programming it is easier to use XOR or Exclusive OR to manipulate bits in hardware registers 
like the example below where we want to disable a peripheral function represented by the 2nd bit position in the
last byte of the register
0000 0111 ^ 0000 0010 = 0000 0101
We could also re-enable the peripheral function represented by the 2nd bit position by ORing
0000 0101 | 0000 0010 = 0000 0111
0xF0 = 1(front=0 or tail=1) 1(forward=0 or reverse=1) 00 0000 1st byte
how does tail forward is then represented in binary and hex = 1000 0000 = 0x80
If speed value 0-256 then we have in hex 0x00 to 0xFF
*****************************************************************************************************************
*/


#include <FastAccelStepper.hpp>
#include <esp32-hal-timer.h>

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
#include <HardwareSerial.h>
#include <ESP32_Servo.hpp>
#include "Wire.h"

#define I2C_DEV_ADDR 0x07

unsigned long previousMillis = 0; // Store the time of the last LED toggle

uint32_t i = 0;

hw_timer_t *pkg1_timer_val = NULL;
hw_timer_t *pod_gate_timer = NULL;


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

int servoValue;


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
char c;
/// @note: 
void onReceive(int len) 
{
  int receive_int=0;
  int count=0;

  Serial.printf("onReceive[%d]: ", len);
  while (Wire.available()) {
    // Original line:
    // Serial.write(Wire.read());
    c = Wire.read();
    receive_int = c << (8 * count) | receive_int;
    count++;
  }
  //Print the Int out.
  Serial.print("Received Number: "); 
  Serial.println(receive_int);
  
  /* The following section is/will be detailed in the corresponding
  /  document: <insert link here>
  /  The ESP32 will react based on the following byte values/messages it receives:
  /
  /     0b 0000 0000 : Return the status of the delivery pod to the user (or companion computer)
  /                    Shows whether the pod is in manual or automated mode.
  /
  /     0b 0000 1111 : Put the delivery pod into automated mode
  /     0b 0000 0001 : deliver package 1
  /     0b 0000 0010 : deliver package 2
  /     0b 0000 0100 : pkg 3
  /     0b 0000 1000 : pkg 4
  /     0b 0001 0000 : Open pod gate
  /     0b 0010 0000 : Close gate
  /     0b 1000 0000 : Completely open the pod package pushers and re-calibrate the stepper motors to 0 (beginning).
  /
  /     0b 1111 0000 : Put the delivery pod into manual mode (would need to establish a set of rules for keyboard values)
  /     0b 0111 0000 : Put the gates into manual mode (accepting commands from a keyboard or some other value)
  /     0b 0000 1100 : Put the front linear actuator into manual mode (gate motor)
  /     0b 0000 0011 : Put tail linear actuator into manual mode
  /     0b 1100 0000 : Put the tail motor into manual mode
  /     0b 0011 0000 : Put the front motor into manual mode
  /     
  /     ^^^ These are all of the byte messages I've thought of so far that we may need but there could be more (or less).
  */
  if (c == '0b00000001')
  {

  }
}


/// @brief 8-26-2024 - a servo value of 110 corresponds to the pod gate completely closing
///        (The linear actuator's values to extend and open the gate)
void IRAM_ATTR close_pod_gate(void)
{
	front_linear_actuator.write(110);    // tell servo to go to position in variable 'pos'
	tail_linear_actuator.write(110);
}


/// @brief 8-26-2024 - a servo value of 23 corresponds to the pod gate completely opening
///        (The linear actuator's values for retracting to open the gate)
void IRAM_ATTR open_pod_gate(void)
{
  
  // // Examples and functions for the esp timer can be found here: https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/timer.html
  // pod_gate_timer = timerBegin(0, 80, true);
  // timerAttachInterrupt(pod_gate_timer, &close_pod_gate, true);

  
  // // This needs its own set of esp-hw timer functions to call for closing the pod gate afterwards
  // // 2nd param needs to be adjusted for how long it takes to completely open / drop off the package.
  // timerAlarmWrite(pod_gate_timer, 1000, true);
  // timerAlarmEnable(pod_gate_timer);

  // Some of the indentation is off whenever I upload from my IDE to Github
	front_linear_actuator.write(23);
	tail_linear_actuator.write(23);
}


/// @brief 8-26-2024 - a servo value of 23 corresponds to the pod gate completely opening
///        (The linear actuator's values for retracting to open the gate)
void IRAM_ATTR open_pod_gate2(void)
{

  // Some of the indentation is off whenever I upload from my IDE to Github
	front_linear_actuator.write(23);
	tail_linear_actuator.write(23);
}


/// @brief Moves package 1 into position for the pod 
/// As of 8-26-2024, I'm only testing the front stepper to move the package
/// In the future, there would be 4 (or perhaps less but more on this later) packages
/// in the pod and so both the tail and front stepper need to move in order to 
/// move package 1 into position.
void IRAM_ATTR move_package1_into_position(void)
{


  /// @note I'm not sure if this section works with interrupts or not
  // pkg1_timer_val = timerBegin(0, 80, true);
  // timerAttachInterrupt(pkg1_timer_val, &open_pod_gate, true);
  // // The second param needs to be adjusted to account for how long
  // // it takes to move the package into position by the stepper motor(s).
  // timerAlarmWrite(pkg1_timer_val, 1000, true);
  // timerAlarmEnable(pkg1_timer_val);

  // stepper->setSpeedInUs(250);  // the parameter is us/step !!!
  // stepper->setAcceleration(500);
  // stepper->runForward();
  // stepper->move(10000);
  // stepper->keepRunning();
  // stepper->runForward();
  // stepper->enableOutputs();
  // stepper->move(10000);
  stepper->reAttachToPin();

}


void setup()
{  
  Serial.begin(115200);
  Serial.println("Starting setup function...");

  Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);

  #if CONFIG_IDF_TARGET_ESP32
    char message[64];
    snprintf(message, 64, "%lu Packets.", i++);
    Wire.slaveWrite((uint8_t *)message, strlen(message));
  #endif

	front_linear_actuator.attach(front_linear_actuator_sig_pin, 1000, 2000); // attaches the servo on pin 18 to the servo object
	tail_linear_actuator.attach(tail_linear_actuator_sig_pin, 1000, 2000); // attaches the servo on pin 18 to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep

	front_linear_actuator.write(110);    // tell servo to go to position in variable 'pos'
	tail_linear_actuator.write(110);

  pinMode(INT0_PIN, OUTPUT);
  // attachInterrupt(digitalPinToInterrupt(INT0_PIN), move_package1_into_position, RISING);
  attachInterrupt(digitalPinToInterrupt(INT0_PIN), open_pod_gate2, RISING);

  pinMode(INT1_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(INT1_PIN), close_pod_gate, RISING);

  /// @todo: (jramayrat) The moment the motors start to move to deliver a particular package,
  ///        I think there needs to be an interrupt to be called (for opening the pod gate) after
  ///        a certain amount of time has passed (corresponding to the amount of time it takes to move
  ///        the package into position). Maybe attach an ISR to a timer / some timer value.
  /// @ref: https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/timer.html
  // timerAttachInterrupt(pkg1_timer_val, &open_pod_gate, true);

  // // xxxxxxxYou shouldn't need these functions. I think they're from a previous version:
  // // timerAlarmWrite(pkg1_timer_val, 1000000, true);
  // // timerAlarmEnable(pkg1_timer_val);

  // timerAttachInterrupt(pod_gate_timer, &close_pod_gate, true);
  // timerAlarmWrite(pod_gate_timer, 1000000, true);
  // timerAlarmEnable(pod_gate_timer);

  engine.init();
  stepper = engine.stepperConnectToPin(front_stepper_step_pin);
  if (stepper) {
    Serial.println("Successfully created stepper object");
    stepper->setDirectionPin(front_stepper_dir_pin);
    stepper->setEnablePin(front_stepper_enable_pin);

    // If auto enable/disable need delays, just add (one or both):
    stepper->setDelayToEnable(50);
    stepper->setDelayToDisable(50);

    /// @note: The value for stepper->move() needs to be adjusted for moving package 1 
    ///        into position. It's another value besides 9000.
    stepper->setSpeedInUs(250);  // the parameter is us/step !!!
    stepper->setAcceleration(500);
    stepper->move(20000);
    // stepper->runForward();
  }
  

}

void loop()
{  
  // Serial.println("loop iteration");

  // Serial.print("Enter value for pkg:");
	// while (Serial.available() == 0){}
	// servoValue = Serial.parseInt();
	// if (servoValue == 0)
	// {
	// 		Serial.println("Moving pkg");
  //     digitalWrite(INT0_PIN, LOW);
  //     delay(1);
  //     digitalWrite(INT0_PIN, HIGH);
  //     delay(1);

	// }


  unsigned long currentMillis = millis(); // Get the current time
  // if (currentMillis - previousMillis < 500) 
  // {    
  // Serial.println("Opening pod gate");
  //      digitalWrite(INT0_PIN, LOW);
  //      delay(1);
  //      digitalWrite(INT0_PIN, HIGH);
  //      delay(1);
  // } else if (currentMillis - previousMillis < 1000) 
  // {
  //   Serial.println("Closing pod gate");
  //      digitalWrite(INT1_PIN, LOW);
  //      delay(1);
  //      digitalWrite(INT1_PIN, HIGH);
  //      delay(1);
  // } else{
  //   previousMillis = currentMillis;
  // }

}
