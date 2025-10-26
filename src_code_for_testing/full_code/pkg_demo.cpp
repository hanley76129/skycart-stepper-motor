#include <FastAccelStepper.hpp>
#include <HardwareSerial.h>
#include <ESP32_Servo.hpp>
 
// Interrupt pins for activating the linear actuators
#define INT0_PIN 13
#define INT1_PIN 14

// 16 servo objects can be created on the ESP32
Servo front_actuonix;  // create servo object to control a servo
Servo tail_actuonix;

int servoValue;
int serial_value = 2;
int loop_iter = 0;
// int pos = 0;    // variable to store the servo position

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int tail_actuonix_pin = 17;
int front_actuonix_pin = 19;

// int pos = 0;    // variable to store the servo position
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

#define dirPinStepper    25
#define enablePinStepper 27
#define stepPinStepper   26


// #define front_stepper_dir_pin    25
// #define front_stepper_step_pin   26
// #define front_stepper_enable_pin 27

/// @brief ISR for entering 0 from serial parsing. 7-31-2024, this is now working without
///        causing the ESP32 to reboot. But it makes me question why someone wrote an entire
///        library on controlling servos via interrupts for the ESP32: https://github.com/khoih-prog/ESP32_ISR_Servo/tree/master
/// @param  
/// @return 
void IRAM_ATTR interrupt_func1(void)
{

	/// @todo: Need to test what values for front actuonix completely closes and opens the gate
	///				 Tests from 8-19-2024: servoValue = 23 for completely open, 110 for completely closed
	tail_actuonix.write(23);    // tell servo to go to position in variable 'pos'
}


void IRAM_ATTR interrupt_func2(void)
{

	/// @todo: Need to test what values for front actuonix completely closes and opens the gate
	///				 Tests from 8-19-2024: servoValue = 23 for completely open, 110 for completely closed
	front_actuonix.write(23);    // tell servo to go to position in variable 'pos'
}


/// @brief This function needs to be able to deliver the first package in the pod.
///        So it should run the the steppers up to a certain distance and we could
///        measure this using a non-blocking timer (e.g. using the millis() command)
///        or maybe attaching it to a pulse counter (the FastAccel stepper library provides this).
void deliver_first_package()
{
      Serial.println("Moving stepper");
      // stepper->move(50000);
      stepper->attachToPulseCounter(QUEUES_MCPWM_PCNT, 0, 0);
      stepper->runForward();

}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup function...");
   engine.init();
   stepper = engine.stepperConnectToPin(stepPinStepper);
   if (stepper) {
      Serial.println("Calling step commands in setup");
      stepper->setDirectionPin(dirPinStepper);
      stepper->setEnablePin(enablePinStepper);
      //stepper->setAutoEnable(true);

      // If auto enable/disable need delays, just add (one or both):
      stepper->setDelayToEnable(50);
      stepper->setDelayToDisable(50);

      // SMALLER values make the stepper go FASTER
      // larger values -> slower stepper
      // Differences need to be in the 10,000's to notice it in the hardware
      stepper->setSpeedInUs(250);  // the parameter is us/step !!!
      
      stepper->setAcceleration(500);
      // stepper->move(10000);
      Serial.println("Did it go");
      // stepper->runForward();
      // stepper->keepRunning();
      //stepper->disableOutputs();



   }
//    delay(10000);
//    digitalWrite(enablePinStepper, LOW);
      pinMode(INT0_PIN, OUTPUT);
      pinMode(INT1_PIN, OUTPUT);

      attachInterrupt(digitalPinToInterrupt(INT0_PIN), interrupt_func1, RISING);
      attachInterrupt(digitalPinToInterrupt(INT1_PIN), interrupt_func2, RISING);


      front_actuonix.attach(front_actuonix_pin, 1000, 2000); // attaches the servo on pin 18 to the servo object
      tail_actuonix.attach(tail_actuonix_pin, 1000, 2000); // attaches the servo on pin 18 to the servo object
	front_actuonix.write(110);    // tell servo to go to position in variable 'pos'
	tail_actuonix.write(110);

}

void loop() {

      Serial.println("Loop iteration");
      loop_iter++;
      // stepper->move(5000);
      if (serial_value == 2)
      {
            Serial.print("Enter 0 to move the stepper");
            while (Serial.available() == 0){}
            serial_value = Serial.parseInt();
            if (serial_value == 0)
            {
                  Serial.println("Moving stepper");
                  // stepper->move(50000);
                  stepper->attachToPulseCounter(QUEUES_MCPWM_PCNT, 0, 0);
                  stepper->runForward();

                  serial_value = 1;

            }
      }

      if (loop_iter == 300)
            Serial.println("Moving servo");
            digitalWrite(INT0_PIN, LOW);
            delay(1);
            digitalWrite(INT0_PIN, HIGH);
            delay(1);

            digitalWrite(INT1_PIN, LOW);
            delay(1);
            digitalWrite(INT1_PIN, HIGH);
            delay(1);
      if (loop_iter == 2000)
      {
            Serial.println("Moving servo");
            digitalWrite(INT0_PIN, LOW);
            delay(1);
            digitalWrite(INT0_PIN, HIGH);
            delay(1);

            digitalWrite(INT1_PIN, LOW);
            delay(1);
            digitalWrite(INT1_PIN, HIGH);
            delay(1);

      }
}