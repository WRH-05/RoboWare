#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 10
#define RST_PIN 9

// Global MFRC522 instance definition
MFRC522 mfrc522(SS_PIN, RST_PIN);

bool runReadMode = true;  // Change to false if you want to run the write code

void setup() {
  Serial.begin(9600);
  while (!Serial);  // optional for boards that wait on serial

  SPI.begin();
  if (runReadMode) {
    setupRFIDRead();   // defined in RFID-read.ino
  } else {
    setupRFIDWrite();  // defined in RFID-write.ino
  }
}

void loop() {
  if (runReadMode) {
    loopRFIDRead();    // defined in RFID-read.ino
  } else {
    loopRFIDWrite();   // defined in RFID-write.ino
  }
}
