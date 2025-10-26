/* This code is for testing how long it takes to open the pod gate and move packages */

#include <FastAccelStepper.h>
#include <HardwareSerial.h>
#include <ESP32_Servo.h>

// #include <ros.h>
// #include <std_msgs/String.h>

// Interrupt pins for activating the linear actuators
#define INT0_PIN 13
#define INT1_PIN 14

// 16 servo objects can be created on the ESP32
Servo front_actuonix;  // create servo object to control a servo
Servo tail_actuonix;

int serial_value = 2;
int loop_iter = 0;
// int pos = 0;    // variable to store the servo position

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int tail_actuonix_pin = 17;
int front_actuonix_pin = 19;

int servoValue;
int front_servo_val;
int all_linear_actuator_vals;

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
void IRAM_ATTR actuonix_ISR(void)
{

      /// Tests from 8-19-2024: servoValue = 23 for completely open, 110 for completely closed
      tail_actuonix.write(servoValue);
      // front_actuonix.write(front_servo_val);    
  
      
}

void IRAM_ATTR actuonix_ISR_front(void)
{

      /// Tests from 8-19-2024: servoValue = 23 for completely open, 110 for completely closed
      front_actuonix.write(front_servo_val);    
      
}


// void IRAM_ATTR actuonix_plus(void)
// {
//       if (servoValue < 90)
//       {
// 	      tail_actuonix.write(servoValue);    
// 	      front_actuonix.write(servoValue);    
//       }
// }




void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup function...");
   engine.init();
   stepper = engine.stepperConnectToPin(stepPinStepper);
   if (stepper) 
   {
      Serial.println("Calling step commands in setup");
      stepper->setDirectionPin(dirPinStepper);

      stepper->setEnablePin(enablePinStepper);
      // stepper->setEnablePin(enablePinStepper, false);
      // stepper->setAutoEnable(true);

      // If auto enable/disable need delays, just add (one or both):
      stepper->setDelayToEnable(50);
      stepper->setDelayToDisable(50);

      // SMALLER values make the stepper go FASTER
      // larger values -> slower stepper
      // Differences need to be in the 10,000's to notice it in the hardware
      stepper->setSpeedInUs(250);  // the parameter is us/step !!!
      
      stepper->setAcceleration(1000);
      // stepper->move(10000);
      Serial.println("Did it go");
      // stepper->runForward();
      // stepper->keepRunning();
      // stepper->disableOutputs();

   }
//    delay(10000);
//    digitalWrite(enablePinStepper, LOW);
      pinMode(INT0_PIN, OUTPUT);
      pinMode(INT1_PIN, OUTPUT);
      attachInterrupt(digitalPinToInterrupt(INT0_PIN), actuonix_ISR, RISING);
      attachInterrupt(digitalPinToInterrupt(INT1_PIN), actuonix_ISR_front, RISING);

      front_actuonix.attach(front_actuonix_pin, 1000, 2000); // attaches the servo on pin 18 to the servo object
      tail_actuonix.attach(tail_actuonix_pin, 1000, 2000); // attaches the servo on pin 18 to the servo object
	
      /// @note: This line should close the pod gates immediately upon starting 
      ///        uploading/rebooting the ESP32
      all_linear_actuator_vals = 110; // test if the conditional works first (24 < value < 110)
      servoValue = all_linear_actuator_vals;
      front_servo_val = all_linear_actuator_vals;
      // front_actuonix.write(all_linear_actuator_vals); 
	// tail_actuonix.write(all_linear_actuator_vals);

}
int incomingByte = 0; // for incoming serial data
int first_open_attempt = 2;
void loop() 
{

      ///////////////////////////////////////////////
      ///////////////////////////////////////////////
      ///////////////////////////////////////////////
      Serial.println("Loop iteration");
      // loop_iter++;
      Serial.println("1: Open pod gate | 2: Close gate | 3: Move stepper to gate | 4: Move stepper away ");
      while (Serial.available() == 0){}
      if (Serial.available() > 0)
      {
            // read the incoming byte:
            // incomingByte = Serial.read();
            // int menuChoice = Serial.parseInt();
            int menuChoice = Serial.read();

            switch (menuChoice)
            {
                  case 49:
                        // if (first_open_attempt == 2)
                        if (servoValue && front_servo_val == 110)
                        {
                             front_servo_val = front_servo_val - 30;

                              Serial.println("Opening front gate a little bit first");
                              digitalWrite(INT1_PIN, LOW);
                              delay(1);
                              digitalWrite(INT1_PIN, HIGH);
                              delay(2000);
                              first_open_attempt = 1;   
                        }
                        if (servoValue || front_servo_val > 23)
                        {
                              Serial.println("Opening gate");
                              if (servoValue > 23)
                              {
                                    servoValue--;

                                    digitalWrite(INT0_PIN, LOW);
                                    delay(1);
                                    digitalWrite(INT0_PIN, HIGH);
                                    delay(1);
                              }
                              if (front_servo_val > 23)
                              {
                                    front_servo_val--;
                                    digitalWrite(INT1_PIN, LOW);
                                    delay(1);
                                    digitalWrite(INT1_PIN, HIGH);
                                    delay(1);
                              }
                              Serial.print("Servo and front servo value: ");
                              Serial.println(servoValue);
                              Serial.println(front_servo_val);
                              // break;
                        } else 
                        {
                              Serial.println("Attempting to move servos past 23");
                              Serial.println("Servo and front servo value: ");
                              Serial.println(servoValue);
                              Serial.println(front_servo_val);
                              // break;
                        }
                        break;
                  case 50:
                        if (servoValue || front_servo_val < 110)
                        {
                              Serial.println("Closing gate");
                              if (servoValue < 110)
                              {
                                    servoValue++;
                                    digitalWrite(INT0_PIN, LOW);
                                    delay(1);
                                    digitalWrite(INT0_PIN, HIGH);
                                    delay(1);
                              }
                              if (front_servo_val < 110)
                              {
                                    front_servo_val++;

                                    digitalWrite(INT1_PIN, LOW);
                                    delay(1);
                                    digitalWrite(INT1_PIN, HIGH);
                                    delay(1);
                              }
                              Serial.println("Servo and front servo value: ");
                              Serial.println(servoValue);
                              Serial.println(front_servo_val);
                              // break;
                        } else
                        {
                              Serial.println("Attempting to move servo past 110");
                              first_open_attempt = 2;
                              Serial.println("Servo and front servo value: ");
                              Serial.println(servoValue);
                              Serial.println(front_servo_val);
                              // break;
                        }
                        break;
 

      Serial.println("1: Open pod gate | 2: Close gate | 3: Move stepper to gate | 4: Move stepper away | 5: Stop motor");
                  case 51:
                        Serial.println("Stepper forward");
                        stepper->runForward();
                        break;
                  case 52:
                        Serial.println("Stepper backward");
                        stepper->runBackward();
                        break;
                  case 53:
                        Serial.println("Stopping stepper");
                        stepper->forceStop();
                        break;

            }
            Serial.println(menuChoice);
      }
}