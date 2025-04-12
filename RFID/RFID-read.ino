#include <MFRC522.h>
extern MFRC522 mfrc522;  // Declare that mfrc522 is defined in your main file

void setupRFIDRead() {
  mfrc522.PCD_Init();
  Serial.println("RFID Read Mode Initialized");
}

void loopRFIDRead() {
  Serial.println("Looking for an RFID tag...");

  // Check if a new card is present in the RFID field
  if (!mfrc522.PICC_IsNewCardPresent()) {
    Serial.println("No new card present. Retrying in 500ms...");
    delay(500);
    return;
  }

  // Try to read the UID from the card
  if (!mfrc522.PICC_ReadCardSerial()) {
    Serial.println("Card detected but failed to read the card serial number. Retrying...");
    delay(500);
    return;
  }

  // If we get here, a card is detected and its UID has been read
  Serial.print("Card UID: ");
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    // Print each byte of the UID in hexadecimal format
    Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(mfrc522.uid.uidByte[i], HEX);
  }
  Serial.println(" (Read successfully)");

  // Halt communication with the current card
  mfrc522.PICC_HaltA();

  // Delay to prevent reading the same card multiple times in rapid succession
  delay(1000);
}
