// https://www.actuonix.com/assets/images/datasheets/ActuonixL12Datasheet.pdf
// https://github.com/jkb-git/ESP32Servo/blob/master/examples/Sweep/Sweep.ino


#include <HardwareSerial.h>

#include <ESP32_Servo.hpp>
 
Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32
 
int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPin = 19;
 
void setup() {
	Serial.begin(115200);
	Serial.println("Starting..");
	// Allow allocation of all timers
	// ESP32PWM::allocateTimer(0);
	// ESP32PWM::allocateTimer(1);
	// ESP32PWM::allocateTimer(2);
	// ESP32PWM::allocateTimer(3);
	// myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep
}
 
void loop() {
 
	Serial.println("Loop iter");
	
	for (pos = 0; pos <= 360; pos += 1) { // goes from 0 degrees to 180 degrees
		// in steps of 1 degree
		myservo.write(0);    // tell servo to go to position in variable 'pos'
		delay(15);             // waits 15ms for the servo to reach the position
		Serial.println(pos);
	}
	for (pos = 360; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
		myservo.write(180);    // tell servo to go to position in variable 'pos'
		delay(15);             // waits 15ms for the servo to reach the position
		Serial.println(pos);

	}
}