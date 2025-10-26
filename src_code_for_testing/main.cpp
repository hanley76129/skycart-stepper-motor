/* ESP32 stepper motor control where commands are accepted from a main PC.

  Author:
    Joshua Ramayrat

  References:
    https://www.makerguides.com/esp32-and-tb6600-stepper-motor-driver/
    https://how2electronics.com/interfacing-pn532-nfc-rfid-module-with-arduino/
    https://github.com/don/NDEF/blob/master/examples/P2P_Receive/P2P_Receive.ino
*/

#include <Arduino.h>
#include <Stepper.h>
#include <string>
#include <HardwareSerial.h>


#include "pn532.h"
#include "pn532_uno.h"

uint8_t buff[255];
uint8_t uid[MIFARE_UID_MAX_LENGTH];
int32_t uid_len = 0;
uint8_t key_a[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t pn532_error = PN532_ERROR_NONE;

PN532 pn532;

int PUL = 25;
int DIR = 26;
int EN  = 27;

#define RX 17
#define TX 18

const byte NumChars = 32;
char ReceivedChars[NumChars];
char TempChars[NumChars];       		      // temporary array for use when parsing

// HardwareSerial Serial2(2);

/// @brief  Takes as input whether to move a package forward (or backward?) and by how far in millimeters
/// @param  direction: Whether we want the package to move forward or backward
/// @param  distance: By how far we want to move the package in millimeters
/// @return We could potentially have a distance sensor / encoder that could return the actual values.
void move_package(std::string direction, float distance);

/// @reference: https://forum.arduino.cc/t/parsing-serial-data-and-separating-it-into-variables/619949/3
void ParseData();

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("NFC Peer to Peer Example - Receive Message");

  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(EN, OUTPUT);

  // put your setup code here, to run once:
  PN532_I2C_Init(&pn532);
  Serial.println("Hello!");
  if (PN532_GetFirmwareVersion(&pn532, buff) == PN532_STATUS_OK) {
    Serial.print("Found PN532 with firmware version: ");
    Serial.print(buff[1], DEC);
    Serial.print(".");
    Serial.println(buff[2], DEC);
    Serial.println("Waiting for RFID/NFC card...");
  } else {
    return;
  }
  PN532_SamConfiguration(&pn532);
  while (1)
  {
    // Check if a card is available to read
    uid_len = PN532_ReadPassiveTarget(&pn532, uid, PN532_MIFARE_ISO14443A, 1000);
    if (uid_len == PN532_STATUS_ERROR) {
      Serial.print(".");
    } else {
      Serial.print("Found card with UID: ");
      for (uint8_t i = 0; i < uid_len; i++) {
        if (uid[i] <= 0xF) {
          Serial.print("0");
        }
        Serial.print(uid[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      break;
    }
  }
  // Write block #6
  uint8_t block_number = 6;
  uint8_t DATA[] = {0x00, 0x01, 0x02, 0x03};
  pn532_error = PN532_Ntag2xxWriteBlock(&pn532, DATA, block_number);
  if (pn532_error) {
    Serial.print("Error: 0x");
    Serial.print(pn532_error, HEX);
    
    return;
  }
  pn532_error = PN532_Ntag2xxReadBlock(&pn532, buff, block_number);
  if (pn532_error) {
    Serial.print("Error: 0x");
    Serial.print(pn532_error, HEX);
    return;
  }
  for (uint8_t i = 0; i < sizeof(DATA); i++) {
    if (DATA[i] != buff[i]) {
      Serial.print("Write block ");
      Serial.print(block_number, DEC);
      Serial.print(" failed\r\n");
      return;
    }
  }
  Serial.print("Write block ");
  Serial.print(block_number, DEC);
  Serial.print(" successfully\r\n");
}

void loop() {
  // put your main code here, to run repeatedly:
}



/// @brief: Moves a physical package. You could really expand on this function
///         like adding diagnostic capabilities to see if a package is in the correct
///         spot or if there were errors. This would make use of sensor information
void move_package(std::string direction, float distance)
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
      digitalWrite(DIR, LOW);
      digitalWrite(EN, HIGH);
      digitalWrite(PUL, HIGH);
      delayMicroseconds(50);
      digitalWrite(PUL, LOW);
      delayMicroseconds(50);
    }
  }
  if (direction == "backward")
  {
    for (int i = 0; i < number_of_stepper_motor_steps; i++)
    {
      digitalWrite(DIR, HIGH);
      digitalWrite(EN,  HIGH);
      digitalWrite(PUL, HIGH);
      delayMicroseconds(50);
      digitalWrite(PUL, LOW);
      delayMicroseconds(50);
    }
  }
}


// void recvWithStartEndMarkers() 
// {
//     static boolean recvInProgress = false;
//     static byte ndx = 0;
//     char startMarker = '<';
//     char endMarker = '>';
//     char rc;

//     while (Serial.available() > 0 && newData == false) {
//         rc = Serial.read();

//         if (recvInProgress == true) {
//             if (rc != endMarker) {
//                 receivedChars[ndx] = rc;
//                 ndx++;
//                 if (ndx >= numChars) {
//                     ndx = numChars - 1;
//                 }
//             }
//             else {
//                 receivedChars[ndx] = '\0'; // terminate the string
//                 recvInProgress = false;
//                 ndx = 0;
//                 newData = true;
//             }
//         }

//         else if (rc == startMarker) {
//             recvInProgress = true;
//         }
//     }
// }


// /// @reference: https://forum.arduino.cc/t/parsing-serial-data-and-separating-it-into-variables/619949/3
// /// @note: this is WIP
// void ParseData()	      			            
// {      // split the data into its parts

//     char * strtokIndx; // this is used by strtok() as an index

//     strtokIndx = strtok(tempChars,",");      // get the first part - the string
//     strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
 
//     strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
//     integerFromPC = atoi(strtokIndx);     // convert this part to an integer

//     strtokIndx = strtok(NULL, ",");
//     floatFromPC = atof(strtokIndx);     // convert this part to a float

// }


// void showParsedData() {
//     Serial.print("Message ");
//     Serial.println(messageFromPC);
//     Serial.print("Integer ");
//     Serial.println(integerFromPC);
//     Serial.print("Float ");
//     Serial.println(floatFromPC);
// }