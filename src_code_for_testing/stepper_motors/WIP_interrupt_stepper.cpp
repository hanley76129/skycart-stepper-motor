/*

  For the electrical wiring/architecture, refer to here: https://docs.google.com/drawings/d/1PiIA1vhQJSO_LVA35kLkUd2acoN4x1MLj0Qvopayqfs/edit

*/

#include <FastAccelStepper.hpp>
#include "Wire.h"
#include <HardwareSerial.h>
#include <iostream>

/// @todo: (jramayrat) Look into this switch library for limit switches: https://github.com/ronbentley1/eazy-switch-library/tree/main
 


/// @note: (jramayrat) The intent behind using this library is to store the 'position' of the stepper motor and the package
///        into permanent memory in case of the ESP32 shutting off and we end up losing knowledge of where the package
///        or stepper motor was last positioned. It's a way of de-risking destroying the package or hardware.
///        Reference: https://randomnerdtutorials.com/esp32-flash-memory/
#include <EEPROM.h>

#define EEPROM_SIZE 1

#define I2C_DEV_ADDR 0x07

#define INT0_PIN 13

#define front_stepper_dir_pin    25
#define front_stepper_enable_pin 27
#define front_stepper_step_pin   26

uint32_t i = 0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

float front_stepper_position = 0;


/// @note: This value is temporary and just for testing the problem of 
///        pulse counting and placing the stepper motor's position into permanent memory:
int front_stepper_move = 9000;
int16_t last_read_pulse_counter = 0;


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

  /// @note: Making this line char* str_from_rpi makes the ESP32 reboot itself and call that guru meditation error.
  std::string str_from_rpi;

  Serial.printf("onReceive[%d]: ", len);
  while (Wire.available()) {
    // Original line:
    // Serial.write(Wire.read());
    byteArray = Wire.read();
    std::cout << "byteArray: "; 
    Serial.println(byteArray);
    str_from_rpi += byteArray;
    count++;
  }

  std::cout << "str_from_rpi: " << str_from_rpi << std::endl;
  char first_char = str_from_rpi[0];

  digitalWrite(INT0_PIN, LOW);
  delay(1);
  digitalWrite(INT0_PIN, HIGH);
  delay(1);

  if (str_from_rpi[0] == '1')
  {

    ////////////////////////////////////////////////////////////////////////////////////
    /// @note: As of 7-23-2024, uncommenting this section results in the ESP32 rebooting:
    /*    
      abort() was called at PC 0x40085837 on core 1

      Backtrace: 0x40084335:0x3ffbf2ac |<-CORRUPTED

      ELF file SHA256: ac7609289ac10d21

      Rebooting...
    */
    // std::cout << "Attempting to call interrupt function" << std::endl;
    digitalWrite(INT0_PIN, LOW);
    delay(1);
    digitalWrite(INT0_PIN, HIGH);
    delay(1);
    ////////////////////////////////////////////////////////////////////////////////////

    Serial.println("in void onReceive function, 'str_from_rpi' equals '1'.");
    Serial.println("Attempting to call stepper functions...");

    stepper->setDirectionPin(front_stepper_dir_pin);
    stepper->setEnablePin(front_stepper_enable_pin);
      
    // std::cout << "Calling 'deliver_first_package'" << std::endl;
    /// @todo: Try to call the functions provided by the FastAccel library in this
    ///        section. Once/if it's working, then try to call this interrupt from
    ///        another function (the onReceive function used for I2C)
    stepper->setDelayToEnable(50);
    stepper->setDelayToDisable(50);


    // Smaller values make the stepper go faster, larger values make it slower
    // Differences need to be in the 10,000's to notice it in the hardware
    stepper->setSpeedInUs(10000);  // the parameter is us/step !!!
    stepper->setAcceleration(1000);
    
    // This value needs to be found empirically. Could also do it using math but finding
    // this number manually could be easier.
    stepper->move(front_stepper_move);
    Serial.println("Finished calling stepper functions from within onReceive's if-statement...");

  } else {
    std::cout << "str_from_rpi not equal to '1'" << std::endl;
  }
}


void IRAM_ATTR deliver_first_package(void)
{
  // std::cout << "Calling ISR 'deliver_first_package'" << std::endl;
    stepper->setDirectionPin(front_stepper_dir_pin);
    stepper->setEnablePin(front_stepper_enable_pin);
      
    // std::cout << "Calling 'deliver_first_package'" << std::endl;
    /// @todo: Try to call the functions provided by the FastAccel library in this
    ///        section. Once/if it's working, then try to call this interrupt from
    ///        another function (the onReceive function used for I2C)
    stepper->setDelayToEnable(50);
    stepper->setDelayToDisable(50);


    // Smaller values make the stepper go faster, larger values make it slower
    // Differences need to be in the 10,000's to notice it in the hardware
    stepper->setSpeedInUs(10000);  // the parameter is us/step !!!
    stepper->setAcceleration(1000);
    
    // This value needs to be found empirically. Could also do it using math but finding
    // this number manually could be easier.
    stepper->move(front_stepper_move);
    // Serial.println("Finished calling stepper functions from within onReceive's if-statement...");

  // stepper->setDirectionPin(front_stepper_dir_pin);
  // stepper->setEnablePin(front_stepper_enable_pin);
  // std::cout << "Calling 'deliver_first_package'" << std::endl;
  /// @todo: Try to call the functions provided by the FastAccel library in this
  ///        section. Once/if it's working, then try to call this interrupt from
  ///        another function (the onReceive function used for I2C)
  // stepper->setDelayToEnable(50);
  // stepper->setDelayToDisable(50);


  // // Smaller values make the stepper go faster, larger values make it slower
  // // Differences need to be in the 10,000's to notice it in the hardware
  // stepper->setSpeedInUs(10000);  // the parameter is us/step !!!
  // stepper->setAcceleration(1000);
  
  // // This value needs to be found empirically. Could also do it using math but finding
  // // this number manually could be easier.
  // stepper->move(9000);
  // Serial.println("Stepper commands were called and now calling delay(10000)");
  // delay(10000);
  // Serial.println("Setting enable pin to low");
  // digitalWrite(front_stepper_enable_pin, LOW);
  
  // stepper->runForward();
  // stepper->keepRunning();

  /// @note: There needs to be some logic/code to re-balance the pod/drone and then disable the motors
  ///         afterwards to prevent the hardware from failing and/or overheating
	
  // delay(40000);             // waits 15ms for the servo to reach the position


}


void setup() 
{

	Serial.begin(115200);
	Serial.println("Starting setup function...");
  Serial.setDebugOutput(true);

  EEPROM.begin(EEPROM_SIZE);

  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);

#if CONFIG_IDF_TARGET_ESP32
  char message[64];
  snprintf(message, 64, "%lu Packets.", i++);
  Wire.slaveWrite((uint8_t *)message, strlen(message));
#endif
	// Allow allocation of all timers
	// ESP32PWM::allocateTimer(0);
	// ESP32PWM::allocateTimer(1);
	// ESP32PWM::allocateTimer(2);
	// ESP32PWM::allocateTimer(3);

  pinMode(INT0_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(INT0_PIN), deliver_first_package, RISING);

  engine.init();
  stepper = engine.stepperConnectToPin(front_stepper_step_pin);
  if (stepper) {
	  Serial.println("Calling stepper commands in setup function");

    stepper->setDirectionPin(front_stepper_dir_pin);
    stepper->setEnablePin(front_stepper_enable_pin);

    /// @note: This is the original command, I haven't finalized what the right values are:
    ///        attachToPulseCounter(uint8_t pcnt_unit, int16_t low_value = -16384, int16_t high_value = 16384);
    stepper->attachToPulseCounter(0, -16384, 16384);


// ///////////////////////////////////
//     // // stepper->setAutoEnable(true);
//     // stepper->setSpeedInHz(2000);
//     // stepper->setLinearAcceleration(0);
//     // stepper->moveTo(7000, true);
//     // // stepper->moveTo(0, true);
//     // stepper->runForward();
//     // stepper->keepRunning();
// //////////////////////////////////////

//     // stepper->setAutoEnable(true);

//     // If auto enable/disable need delays, just add (one or both):
//     stepper->setDelayToEnable(50);
//     stepper->setDelayToDisable(50);


//     // Smaller values make the stepper go faster, larger values make it slower
//     // Differences need to be in the 10,000's to notice it in the hardware
//     stepper->setSpeedInUs(500);  // the parameter is us/step !!!
    
    
//     stepper->setAcceleration(1000);
//     stepper->move(9000);
//     // stepper->runForward();
//     // stepper->keepRunning();
//     stepper->disableOutputs();


  }
  Serial.println("Stepper functions were called from setup function");
  // delay(10000);
  // digitalWrite(enablePinStepper, LOW);


}


void loop() 
{
  // last_read_pulse_counter = stepper->readPulseCounter();

  // Serial.print("last_read_pulse_counter: ");
  // Serial.println(last_read_pulse_counter);

  // if (last_read_pulse_counter = front_stepper_move)
  // {
  //   Serial.println("(FastAccel library) Pulse counter val = front_stepper_move var");
  // }

  
  Serial.print("Please enter 1 or 0 to open or close: ");

	while (Serial.available() == 0){}
	// int *menuChoice;
	// menuChoice = new int;
	// *menuChoice = Serial.parseInt();

	int menuChoice = Serial.parseInt();
  
	switch (menuChoice)
	{
		case 0:
			Serial.println("Dropping off package 1");
      digitalWrite(INT0_PIN, LOW);
      delay(1);
      digitalWrite(INT0_PIN, HIGH);
      delay(40000);

			break;
  }
}
