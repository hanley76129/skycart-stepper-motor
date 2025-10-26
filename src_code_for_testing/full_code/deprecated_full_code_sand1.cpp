// https://www.actuonix.com/assets/images/datasheets/ActuonixL12Datasheet.pdf
// https://github.com/jkb-git/ESP32Servo/blob/master/examples/Sweep/Sweep.ino
// https://deepbluembedded.com/arduino-software-interrupts/

#include <HardwareSerial.h>
#include <ESP32_Servo.hpp>
 
#define INT0_PIN 13
#define INT1_PIN 14

int PUL1 = 25;
int DIR1 = 26;
int EN1  = 27;

int PUL2 = 28;
int DIR2 = 29;
int EN2  = 30;

// 16 servo objects can be created on the ESP32
Servo short_side_linear_actuator;  // create servo object to control a servo
Servo long_side_linear_actuator;

// int pos = 0;    // variable to store the servo position

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int long_side_actuonixPin = 17;
int short_side_actuonixPin = 19;

// int menuChoice = 0;
int position = 0;

/// @brief: Moves a physical package. You could really expand on this function
///         like adding diagnostic capabilities to see if a package is in the correct
///         spot or if there were errors. This would make use of sensor information
void actuate_stepper_motor(std::string direction, float distance)
{
  /// @note: Without knowing the mechanical characteristics of the screw to which
  ///        the stepper motor is attached, we don't have an exact relation between
  ///        the number of steps the stepper motor should take and how far of a distance
  ///        the package has actually travelled.
  float number_of_stepper_motor_steps = distance;
  if (direction == "forward")
  {
    for (int i = 0; i < number_of_stepper_motor_steps; i++)
    {
      digitalWrite(DIR1, LOW);
      digitalWrite(EN1, HIGH);
      digitalWrite(PUL1, HIGH);
      delayMicroseconds(50);
      digitalWrite(PUL1, LOW);
      delayMicroseconds(50);
    }
  }
  if (direction == "backward")
  {
    for (int i = 0; i < number_of_stepper_motor_steps; i++)
    {
      digitalWrite(DIR1, HIGH);
      digitalWrite(EN1,  HIGH);
      digitalWrite(PUL1, HIGH);
      delayMicroseconds(50);
      digitalWrite(PUL1, LOW);
      delayMicroseconds(50);
    }
  }
}

/// @brief: Moves a physical package. You could really expand on this function
///         like adding diagnostic capabilities to see if a package is in the correct
///         spot or if there were errors. This would make use of sensor information
void IRAM_ATTR move_package_one(void)
{
  /// @note: Without knowing the mechanical characteristics of the screw to which
  ///        the stepper motor is attached, we don't have an exact relation between
  ///        the number of steps the stepper motor should take and how far of a distance
  ///        the package has actually travelled.
  float number_of_stepper_motor_steps = 100;	// arbitrary number but would need to find this
																							// manually/by trial-and-error on the real system.
	std::string direction = "forward";	// forward for now

  if (direction == "forward")
  {
    for (int i = 0; i < number_of_stepper_motor_steps; i++)
    {
      digitalWrite(DIR1, LOW);
      digitalWrite(EN1, HIGH);
      digitalWrite(PUL1, HIGH);

      delayMicroseconds(50);
      digitalWrite(PUL1, LOW);
      delayMicroseconds(50);
    
		}
  }
}

void IRAM_ATTR close_pod(void)
{

	short_side_linear_actuator.write(360);    // tell servo to go to position in variable 'pos'
	long_side_linear_actuator.write(360);    

	delay(40000);             // waits __ ms for the servo to reach the position
	// Serial.println(pos);

}

void IRAM_ATTR open_pod()
{
	short_side_linear_actuator.write(0);    // tell servo to go to position in variable 'pos'
	long_side_linear_actuator.write(0);     // tell servo to go to position in variable 'pos'

	delay(40000);             // waits __ ms for the servo to reach the position
	// Serial.println(pos);
}

void setup()
{
	Serial.begin(9600);
	Serial.println("Starting..");
	// Allow allocation of all timers
	// ESP32PWM::allocateTimer(0);
	// ESP32PWM::allocateTimer(1);
	// ESP32PWM::allocateTimer(2);
	// ESP32PWM::allocateTimer(3);
	// myservo.setPeriodHertz(50);    // standard 50 hz servo
	short_side_linear_actuator.attach(short_side_actuonixPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
	long_side_linear_actuator.attach(long_side_actuonixPin, 1000, 2000); // attaches the servo on pin 18 to the servo object

	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep

  pinMode(INT0_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(INT0_PIN), close_pod, RISING);
  pinMode(INT1_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(INT1_PIN), open_pod, RISING);

}
 
void loop() {
  Serial.print("Please enter 5 or 6 to open or close: ");

	while (Serial.available() == 0){}
	// int *menuChoice;
	// menuChoice = new int;
	// *menuChoice = Serial.parseInt();

	int menuChoice = Serial.parseInt();


	// if (menuChoice = 0)
	// {
	// 	Serial.println("Closing Package dropoff system");
	// 	digitalWrite(INT0_PIN, LOW);
	// 	delay(1);
	// 	digitalWrite(INT0_PIN, HIGH);
	// }
	// if (menuChoice = 1)
	// {
	// 	Serial.println("Opening package dropoff system");
	// 	digitalWrite(INT1_PIN, LOW);
	// 	delay(1);
	// 	digitalWrite(INT1_PIN, HIGH);
	// }

	
	switch (menuChoice)
	{
		case 5:
			Serial.println("Closing Package dropoff system");
      digitalWrite(INT0_PIN, LOW);
      delay(1);
      digitalWrite(INT0_PIN, HIGH);
      delay(40000);

			break;

		case 6:
			Serial.println("Opening package dropoff system");
      digitalWrite(INT1_PIN, LOW);
      delay(1);
      digitalWrite(INT1_PIN, HIGH);
      delay(40000);

			break;
		

		// @jramayrat: I think this would involve calling several interrupt service 
		//             routines in sequence, where we must call an ISR that moves the package to the
		//             gate/linear actuator location using the stepper motors; then another ISR
		//						 that retracts the actuators to open the pod; then another ISR that
		//						 extends the actuators to close the pod; then another ISR that moves/bunches
		//             the packages together again; then perhaps another ISR that moves the packages
		//						 to a stable location with regards to the drone's center of gravity to maintain
		//             flight stability. 
		//
		//             Given the mechanical design of the motor/screw system, I believe we can only
		//             drop off packages in a certain order. Or at least it depends on the number
		//             of packages in the pod and/or what pod bay they are located in. As an example,
		//             assuming that the gate/exit of the pod is closer to where package 1 is and we have
		//             a full pod, if we wanted to drop off package number 4, we would not be able to do so 
		//             because the motor/screw cannot extend far enough to move packages 1, 2, and 3 out 
		//             of the way. Package 4 is located at the opposite end of package number 1 and far 
		//             away from the gate/exit.
		//             
		//             In other words, I think there needs to be some sort of system in place to know 
		//             what packages were dropped off and the pod needs to know how many packages are in its bay
		//             to begin with. We may need to store this type of information in a few variables and
		//             the delivery system would act accordingly. It could even alert the user if they're trying
		//             to make "invalid deliveries". If people are storing packages within the pod manually,
		//             then there may also need to be a system in place to tell if they are stored in the 
		//             correct order. (Note to self: need to discuss this with Simon)
		//             
		case 1:
			Serial.println("Delivering package one: ");
	}


	// delete menuChoice;

}

