//// This works but only when the actuator is connected to the 5V and ground pin of the ESP32/microcontroller
//// but not when it's connected to an external power supply (7-31-2024)


// https://www.actuonix.com/assets/images/datasheets/ActuonixL12Datasheet.pdf
// https://github.com/jkb-git/ESP32Servo/blob/master/examples/Sweep/Sweep.ino
// https://deepbluembedded.com/arduino-software-interrupts/


// @todo: https://docs.platformio.org/en/latest/core/userguide/device/cmd_monitor.html#filters
//				https://github.com/platformio/platform-espressif8266/issues/31
//				Need to fix issue with the esp32 rebooting
//        https://github.com/esp8266/Arduino/issues/6310
//        https://github.com/platformio/platform-espressif32/issues/105

#include <HardwareSerial.h>
#include <ESP32_Servo.hpp>
 
// Interrupt pins for activating the linear actuators
#define INT0_PIN 13
#define INT1_PIN 14

// 16 servo objects can be created on the ESP32
Servo front_actuonix;  // create servo object to control a servo
Servo tail_actuonix;

// int pos = 0;    // variable to store the servo position

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int tail_actuonix_pin = 17;
int front_actuonix_pin = 19;

// int pos = 0;    // variable to store the servo position

int servoValue;
int position = 0;


/// @brief ISR for entering 0 from serial parsing. 7-31-2024, this is now working without
///        causing the ESP32 to reboot. But it makes me question why someone wrote an entire
///        library on controlling servos via interrupts for the ESP32: https://github.com/khoih-prog/ESP32_ISR_Servo/tree/master
/// @param  
/// @return 
void IRAM_ATTR interrupt_func1(void)
{

	/// @todo: Need to test what values for front actuonix completely closes and opens the gate
	///				 Tests from 8-19-2024: servoValue = 23 for completely open, 110 for completely closed
	tail_actuonix.write(servoValue);    // tell servo to go to position in variable 'pos'
	front_actuonix.write(servoValue);    // tell servo to go to position in variable 'pos'


	/// @todo: Need to test separate values for tail actuonix for opening/closing.
	// tail_actuonix.write(360);
	
	/// @note: Having a delay function in an ISR is not good apparently (and I think serial prints are also bad), 
	///				 according to these references:
	///								https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
	////              https://github.com/khoih-prog/ESP32_ISR_Servo/blob/master/README.md 
	// delay(40000);             // waits 15ms for the servo to reach the position	
	// Serial.println(pos);

}

void setup()
{
	Serial.begin(115200);
	Serial.println("Starting..");
	// Allow allocation of all timers
	// ESP32PWM::allocateTimer(0);
	// ESP32PWM::allocateTimer(1);
	// ESP32PWM::allocateTimer(2);
	// ESP32PWM::allocateTimer(3);

  pinMode(INT0_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(INT0_PIN), interrupt_func1, RISING);
	front_actuonix.attach(front_actuonix_pin, 1000, 2000); // attaches the servo on pin 18 to the servo object
	tail_actuonix.attach(tail_actuonix_pin, 1000, 2000); // attaches the servo on pin 18 to the servo object

}
 
void loop() {

	/////////////////////////////////////////////////////////////
	
	// int servoDistance = 180;
	// Serial.println("Writing to servo");
	// front_actuonix.write(servoDistance);
	// front_actuonix.write(180);



	/////////////////////////////////////////////////////////////
  Serial.print("Enter value for servo:");
	while (Serial.available() == 0){}
	servoValue = Serial.parseInt();
	if (servoValue)
	{
			Serial.println("Moving servo");
      digitalWrite(INT0_PIN, LOW);
      delay(1);
      digitalWrite(INT0_PIN, HIGH);
      delay(1);

	}


  // Serial.print("Please enter 1 or 0 to open or close: ");
	// while (Serial.available() == 0){}
	// int menuChoice = Serial.parseInt();
	// switch (menuChoice)
	// {
	// 	case 0:
	// 		Serial.println("Dropping off package 1");
  //     digitalWrite(INT0_PIN, LOW);
  //     delay(1);
  //     digitalWrite(INT0_PIN, HIGH);
  //     delay(40000);

	// 		break;
	// 	case 1:
	// 		Serial.println("Opening package dropoff system");
  //     digitalWrite(INT1_PIN, LOW);
  //     delay(1);
  //     digitalWrite(INT1_PIN, HIGH);
  //     delay(40000);

	// 		break;
	// }


	// delete menuChoice;



	/////////////////
  // Serial.print("Please enter 1 or 0 to open or close: ");

	// while (Serial.available() == 0){}
	// int *menuChoice;
	// menuChoice = new int;
	// *menuChoice = Serial.parseInt();

	// int menuChoice = Serial.parseInt();
	// // if (menuChoice = 0)
	// // {
	// // 	Serial.println("Closing Package dropoff system");
	// // 	digitalWrite(INT0_PIN, LOW);
	// // 	delay(1);
	// // 	digitalWrite(INT0_PIN, HIGH);
	// // }
	// // if (menuChoice = 1)
	// // {
	// // 	Serial.println("Opening package dropoff system");
	// // 	digitalWrite(INT1_PIN, LOW);
	// // 	delay(1);
	// // 	digitalWrite(INT1_PIN, HIGH);
	// // }

}