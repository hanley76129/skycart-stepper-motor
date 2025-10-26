/*
  References:
    https://github.com/espressif/arduino-esp32/issues/741
    https://esp32.com/viewtopic.php?t=37318
    https://stackoverflow.com/questions/77490525/pn532-sometimes-not-responding-with-esp32
    https://warlord0blog.wordpress.com/2021/10/09/esp32-and-nfc-over-i2c/
    https://www.reddit.com/r/esp32/comments/hgwwr4/has_anyone_managed_to_get_i2c_working_with_the/
    https://www.reddit.com/r/esp32/comments/17uz4sd/weird_issue_with_pn532_nfc_reader/
    https://www.reddit.com/r/esp32/comments/18vdvpu/esp32_pn532/
*/

// esp32-wroom-32 devkit v1 default pins
//       SCL D22
//       SDA D21

// Define the interface type
// #if 0
// #include <SPI.h>
// #include <PN532_SPI.h>
// #include "PN532.h"
// PN532_SPI pn532spi(SPI, 10);
// PN532 nfc(pn532spi);

// #elif 0
// #include <PN532_HSU.h>
// #include <PN532.h>
// PN532_HSU pn532hsu(Serial1);
// PN532 nfc(pn532hsu);

// #else
#include <Wire.h>
#include <PN532_I2C.hpp>
#include <PN532.h>
#include <NfcAdapter.h>

#define PN532_SCL       22
#define PN532_SDA       21  

// #define PN532_SCL       35
// #define PN532_SDA       34  


#define PN532_IRQ_pin   14
#define PN532_RST_pin   25

PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);
// #endif

volatile bool connected = false;


/////////////////////////////////////////////////////////////////////
//    https://github.com/espressif/arduino-esp32/issues/741     /////
/////////////////////////////////////////////////////////////////////
void scan(){
Serial.println(" Scanning I2C Addresses");
uint8_t cnt=0;
for(uint8_t i=0;i<128;i++){
  Wire.beginTransmission(i);
  uint8_t ec=Wire.endTransmission(true);
  if(ec==0){
    if(i<16)Serial.print('0');
    Serial.print(i,HEX);
    cnt++;
  }
  else Serial.print("..");
  Serial.print(' ');
  if ((i&0x0f)==0x0f)Serial.println();
  }
Serial.print("Scan Completed, ");
Serial.print(cnt);
Serial.println(" I2C Devices found.");
}

bool i2cReady(uint8_t adr){
uint32_t timeout=millis();
bool ready=false;
while((millis()-timeout<100)&&(!ready)){
	Wire.beginTransmission(adr);
	ready=(Wire.endTransmission()==0);
	}
return ready;
}

void eepromSize(){
Serial.println("Discovering eeprom sizes 0x50..0x57");
uint8_t adr=0x50,i;
uint16_t size;
char buf[64];
while(adr<0x58){
	i=0;
	size = 0x1000; // Start at 4k
	i += sprintf_P(&buf[i],PSTR("0x%02X: "),adr);
	if(i2cReady(adr)) { // EEPROM answered
		uint8_t zeroByte;
		Wire.beginTransmission(adr);
		Wire.write((uint8_t)0); // set address ptr to 0, two bytes High
		Wire.write((uint8_t)0); // set address ptr to 0, two bytes Low
		uint8_t err=Wire.endTransmission();
		if(err==0){// worked
		  err=Wire.requestFrom(adr,(uint8_t)1);
			if(err==1){// got the value of the byte at address 0
				zeroByte=Wire.read();
				uint8_t saveByte,testByte;
				do{
					if(i2cReady(adr)){
						Wire.beginTransmission(adr);
						Wire.write(highByte(size)); // set next test address
						Wire.write(lowByte(size));
						Wire.endTransmission();
						err=Wire.requestFrom(adr,(uint8_t)1);
						if(err==1){
							saveByte=Wire.read();
							Wire.beginTransmission(adr);
							Wire.write(highByte(size)); // set next test address
							Wire.write(lowByte(size));
							Wire.write((uint8_t)~zeroByte); // change it
							err=Wire.endTransmission();
							if(err==0){ // changed it
								if(!i2cReady(adr)){
									i+=sprintf_P(&buf[i],PSTR(" notReady2.\n"));
									Serial.print(buf);
									adr++;
									break;
									}
								Wire.beginTransmission(adr);
								Wire.write((uint8_t)0); // address 0 byte High
								Wire.write((uint8_t)0); // address 0 byte Low
								err=Wire.endTransmission();
								if(err==0){
									err=Wire.requestFrom(adr,(uint8_t)1);
									if(err==1){ // now compare it
									  testByte=Wire.read();
										}
									else {
										testByte=~zeroByte; // error out
										}
									}
								else {
									testByte=~zeroByte;
									}
								}
							else {
								testByte = ~zeroByte;
								}
							//restore byte
							if(!i2cReady(adr)){
								i+=sprintf_P(&buf[i],PSTR(" notReady4.\n"));
								Serial.print(buf);
								adr++;
								break;
								}
							
							Wire.beginTransmission(adr);
							Wire.write(highByte(size)); // set next test address
							Wire.write(lowByte(size));
							Wire.write((uint8_t)saveByte); // restore it
							Wire.endTransmission();
							}
						else testByte=~zeroByte;
						}
					else testByte=~zeroByte;
					if(testByte==zeroByte){
						size = size <<1;
						}
					}while((testByte==zeroByte)&&(size>0));
				if(size==0) i += sprintf_P(&buf[i],PSTR("64k Bytes"));
				else i+=sprintf_P(&buf[i],PSTR("%dk Bytes"),size/1024);
				if(!i2cReady(adr)){
					i+=sprintf_P(&buf[i],PSTR(" notReady3.\n"));
					Serial.print(buf);
					adr++;
					continue;
					}
				Wire.beginTransmission(adr);
				Wire.write((uint8_t)0); // set address ptr to 0, two bytes High
				Wire.write((uint8_t)0); // set address ptr to 0, two bytes Low
				Wire.write(zeroByte);  //Restore
				err=Wire.endTransmission();
				}
			else i+=sprintf_P(&buf[i],PSTR("Read 0 Failure"));
			}
		else i+=sprintf_P(&buf[i],PSTR("Write Adr 0 Failure"));
			
	  }
	else i+=sprintf_P(&buf[i],PSTR("Not Present."));
	Serial.println(buf);
	adr++;
	}
}
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

void resetPN532(int PN532_IRQ_Pin, int PN532_RSTO_Pin) 
{
  Serial.println("Attempting to reset PN532 device");
  digitalWrite(PN532_IRQ_Pin, LOW);
  delay(100);
  digitalWrite(PN532_IRQ_Pin, HIGH);
  delay(100);
  digitalWrite(PN532_RSTO_Pin, LOW);
  delay(100);
  digitalWrite(PN532_RSTO_Pin, HIGH);
}


bool init_and_connect_NFC() 
{
  // Initialize the I2C bus with the correct SDA and SCL pins
  Wire.begin(PN532_SDA, PN532_SCL);
  Wire.setClock(10000);
  Wire.setTimeOut(2100);
  
  PN532_I2C pn532i2c(Wire);
  PN532 nfc(pn532i2c);

  nfc.begin();

  // Connected, show version
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata)
  {
    Serial.println("PN53x card not found!");
    return false;
  }

  //port
  Serial.print("Found chip PN5"); Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print("Firmware version: "); Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print('.'); Serial.println((versiondata >> 8) & 0xFF, DEC);

  // Set the max number of retry attempts to read from a card
  // This prevents us from waiting forever for a card, which is
  // the default behaviour of the PN532.
  nfc.setPassiveActivationRetries(0xFF);

  // configure board to read RFID tags
  nfc.SAMConfig();

  Serial.println("Waiting for card (ISO14443A Mifare)...");
  Serial.println("");

  return true;
}

void setup(void)
{
  //////////////////// https://github.com/esp8266/Arduino/issues/1025
  //  Serial.println("Starting I2C bus recovery");
  // delay(2000);
  // //try i2c bus recovery at 100kHz = 5uS high, 5uS low
  // pinMode(SDAPIN, OUTPUT);//keeping SDA high during recovery
  // digitalWrite(SDAPIN, HIGH);
  // pinMode(CLKPIN, OUTPUT);
  // for (int i = 0; i < 10; i++) { //9nth cycle acts as NACK
  //   digitalWrite(CLKPIN, HIGH);
  //   delayMicroseconds(5);
  //   digitalWrite(CLKPIN, LOW);
  //   delayMicroseconds(5);
  // }

  // //a STOP signal (SDA from low to high while CLK is high)
  // digitalWrite(SDAPIN, LOW);
  // delayMicroseconds(5);
  // digitalWrite(CLKPIN, HIGH);
  // delayMicroseconds(2);
  // digitalWrite(SDAPIN, HIGH);
  // delayMicroseconds(2);
  // //bus status is now : FREE

  // Serial.println("bus recovery done, starting scan in 2 secs");
  // //return to power up mode
  // pinMode(SDAPIN, INPUT);
  // pinMode(CLKPIN, INPUT);
  // delay(2000);
  // //pins + begin advised in https://github.com/esp8266/Arduino/issues/452
  // Wire.pins(SDAPIN, CLKPIN); //this changes default values for sda and clock as well
  // Wire.begin(SDAPIN, CLKPIN);
  // //only pins: no signal on clk and sda
  // //only begin: no signal on clk, no signal on sda


  // //no further processing in case of error
  // while(true)
  // {
  //   i2c_scan(); 
  // }


  Serial.begin(115200);

  Serial.println("*** Testing Module PN532 NFC RFID ***");
  // Initialize the I2C bus with the correct SDA and SCL pins
  Wire.begin(PN532_SDA, PN532_SCL);
  Wire.setClockStretchLimit(2000);
  Wire.setClock(10000);
  Wire.setTimeOut(2100);

  // https://github.com/espressif/arduino-esp32/issues/741
  scan();
  Serial.println();
  eepromSize();
  /////

  // resetPN532(PN532_IRQ_pin, PN532_RST_pin);

  // PN532_I2C pn532i2c(Wire);
  // PN532 nfc(pn532i2c);
}

void loop(void)
{
  // boolean success;
  // // Buffer to store the UID
  // uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
  // // UID size (4 or 7 bytes depending on card type)
  // uint8_t uidLength;

  // while (!connected) {
  //   connected = init_and_connect_NFC();
  // }

  // // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // // 'uid' will be populated with the UID, and uidLength will indicate
  // // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  // success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);

  // // If the card is detected, print the UID
  // if (success)
  // {
  //   Serial.println("Card Detected");
  //   Serial.print("Size of UID: "); Serial.print(uidLength, DEC);
  //   Serial.println(" bytes");
  //   Serial.print("UID: ");
  //   for (uint8_t i = 0; i < uidLength; i++)
  //   {
  //     Serial.print(" 0x"); Serial.print(uid[i], HEX);
  //   }
  //   Serial.println("");
  //   Serial.println("");
    
  //   delay(1000);
  //   connected = init_and_connect_NFC();
  // }
  // else
  // {
  //   // PN532 probably timed out waiting for a card
  //   // Serial.println("Timed out waiting for a card");
  // }
}

