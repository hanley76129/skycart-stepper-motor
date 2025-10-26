// Receive a NDEF message from a Peer
// Requires SPI. Tested with Seeed Studio NFC Shield v2

// #include "emulatetag.h"
#include "D:\Projects\Skycart\nimbus_software\Multi-payload\esp32-multi-payload\lib\PN532\emulatetag.h"
// #include "NdefMessage.h"
#include "D:\Projects\Skycart\nimbus_software\Multi-payload\esp32-multi-payload\lib\NDEF\NdefMessage.h"
// #include <avr/wdt.h>

// #include <SPI.h>
// #include <PN532_SPI.h>
#include "PN532.h"

#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include <NfcAdapter.h>
#include "Arduino.h"
#include "snep.h"

// #include <HardwareSerial.h>

#define NFC_IRQ 14
#define NFC_RST 25


PN532_I2C pn532i2c(Wire);

SNEP nfc(pn532i2c);
uint8_t ndefBuf[128];


void setup() {

    Serial.begin(9600);
    Serial.println("NFC Peer to Peer Example - Receive Message");
    pinMode(NFC_IRQ, INPUT);
    pinMode(NFC_RST, OUTPUT);
    digitalWrite(NFC_RST, LOW);
    vTaskDelay(210);
    // Set the RST pin to HIGH to finish the reset
    digitalWrite(NFC_RST, HIGH);

}

void loop() {
    Serial.println("Waiting for message from Peer");
    int msgSize = nfc.read(ndefBuf, sizeof(ndefBuf));
    if (msgSize > 0) {
        NdefMessage msg  = NdefMessage(ndefBuf, msgSize);
        msg.print();
        Serial.println("\nSuccess");
    } else {
        Serial.println("Failed");
    }
    delay(3000);
}

