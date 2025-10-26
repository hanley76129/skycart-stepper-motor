/******************THIS ONE WORKS ON THE ESP32 *******************/

/// @ref: https://forum.arduino.cc/t/use-pn532-nfc-reader-with-interrupt/967912/6
///  ^^^^ this one is good

// https://lastminuteengineers.com/handling-esp32-gpio-interrupts-tutorial/
// https://www.upesy.com/blogs/tutorials/what-are-interrupts-in-esp32-with-examples-for-arduino-code#:~:text=Use%20on%20the%20ESP32&text=We%20can%20use%20any%20GPIO%20pin%20for%20interrupts.&text=It%20is%20recommended%20to%20add,that%20the%20function%20runs%20faster.
// https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/peripherals/i2c.html#i2c-api-interrupt-handling
// https://arduino.stackexchange.com/questions/9388/reading-i2c-sensors-with-a-timer-interrupt

#include "emulatetag.h"
// #include "D:\Projects\Skycart\nimbus_software\Multi-payload\esp32-multi-payload\lib\PN532\emulatetag.h"
#include "NdefMessage.h"
// #include "D:\Projects\Skycart\nimbus_software\Multi-payload\esp32-multi-payload\lib\NDEF\NdefMessage.h"
// #include <avr/wdt.h>

// #include <SPI.h>
// #include <PN532_SPI.h>
#include "PN532.h"

#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include <NfcAdapter.h>
#include "Arduino.h"
// #include <HardwareSerial.h>

PN532_I2C pn532i2c(Wire);

int ledpin1 = 5;

// NfcAdapter nfc = NfcAdapter(pn532i2c);

// String tagId1 = "FA 5F 99 1A";
// String tagId2 = "39 0B B6 B0";

// String tagId = "None";
// byte nuidPICC[4];

// void IRAM_ATTR readNFC_ISR()
// {
//   // Serial.println("nothing yet ");
//   readNFC();
// }

// void readNFC() 
// {
//   Serial.println("nothing yet");
//   if (nfc.tagPresent())
//   {
//     Serial.println("FOUND ONE");
//     NfcTag tag = nfc.read();
//     tag.print();
//     tagId = tag.getUidString();
//     Serial.println("Tag id");
//     Serial.println(tagId);
//     digitalWrite(ledpin1, HIGH);
//   }
//   delay(1000);
// }

// void setup()
// {
//   // I attached the PN532's IRQ pin to GPIO pin 2 on the ESP32
//   // const byte interruptPin = 14;
//   // int IRQ_pin = 2;
//   // int PN532_IRQ_pin = 14;
  
//   Serial.begin(115200);
//   pinMode(ledpin1,OUTPUT);
//   Serial.println("System initialized");

//   // Note: The PN532 has its own IRQ pin - https://forum.arduino.cc/t/use-pn532-nfc-reader-with-interrupt/967912
//   // reference: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
//   // pinMode(PN532_IRQ_pin, INPUT);
//   // attachInterrupt(digitalPinToInterrupt(interruptPin), readNFC_ISR, CHANGE);
//   // attachInterrupt(PN532_IRQ_pin, readNFC_ISR, FALLING);


//   // nfc.begin();
//   // digitalWrite(ledpin1, LOW);

//   // "1. The master sends the start condition to every connected slave by switching the SDA line from a high voltage level to a low voltage level before switching the SCL line from high to low:""
//   // https://www.circuitbasics.com/basics-of-the-i2c-communication-protocol/
//   // https://lastminuteengineers.com/handling-esp32-gpio-interrupts-tutorial/
//   /*
//     LOW	Triggers the interrupt whenever the pin is LOW
//     HIGH	Triggers the interrupt whenever the pin is HIGH
//     CHANGE	Triggers the interrupt whenever the pin changes value, from HIGH to LOW or LOW to HIGH
//     FALLING	Triggers the interrupt when the pin goes from HIGH to LOW
//     RISING	
//   */
// }
 
// void loop()
// {
//   // Serial.println("Hello World!");
//   // Serial.println("Hello testing");
//   // readNFC();
//   // digitalWrite(ledpin1, LOW);
//   // delay(1000);

// }

/// @note: It was originally 2 in the reference code, but may need to try other numbers
#define PN532IRQPIN (14)
#define NFC_RST 25

volatile boolean cState = false;

PN532 nfc(pn532i2c);

void cardreading();

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("\nHello!");
  
  // pinMode(PN532IRQPIN, INPUT);
  // pinMode(NFC_RST, OUTPUT);
  // digitalWrite(NFC_RST, LOW);
  // vTaskDelay(210);
  // // Set the RST pin to HIGH to finish the reset
  // digitalWrite(NFC_RST, HIGH);

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();

  if (! versiondata) 
  {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }

  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  
  Serial.println(cState);
  attachInterrupt(digitalPinToInterrupt(PN532IRQPIN), cardreading, FALLING);
  //It generates interrupt, I do not really know why?!
  nfc.SAMConfig();
}

void loop() 
{
  // put your main code here, to run repeatedly:
  if (cState)
  {
    
    Serial.println("Interrupted");
    

    uint8_t success;
    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
    uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
    
    // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
    // 'uid' will be populated with the UID, and uidLength will indicate
    // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
    
    if (success)
    {
      // Display some basic information about the card
      Serial.println("Found an ISO14443A card");
      Serial.print("  UID Length: ");
      Serial.print(uidLength, DEC);
      Serial.println(" bytes");
      Serial.print("  UID Value: ");
      nfc.PrintHex(uid, uidLength);
      
    }
    //This must be called or IRQ won't work!
    nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);
    cState = false;
    Serial.print("OUT: ");
    Serial.println(cState);
  }
  
}
void cardreading()
{
  
  cState = true;
  
}

