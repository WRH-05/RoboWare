#include <MFRC522.h>
extern MFRC522 mfrc522;  // Declare that mfrc522 is defined in RFID.ino

// Default key for authentication
MFRC522::MIFARE_Key key = { {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} };

void setupRFIDWrite() {
  mfrc522.PCD_Init();
  Serial.println("RFID Write Mode Initialized");
}

void loopRFIDWrite() {
  if (!mfrc522.PICC_IsNewCardPresent()) {
    Serial.println("No card present...");
    delay(500); // Wait half a second before checking again
    return;
  }

  if (!mfrc522.PICC_ReadCardSerial()) {
    Serial.println("Failed to read card serial");
    delay(500); // Pause and try again
    return;
  }

  // If we get here, a card is detected
  Serial.println("Card detected, attempting authentication...");
  
  byte block = 4;  // The block to write to
  byte dataBlock[16] = {
    'H', 'e', 'l', 'l', 'o', ' ', 'W', 'I', 'Z', 'A', 'R', 'D', 'S', '!', ' ', ' '
  };

  MFRC522::StatusCode status;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Authentication failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }
  
  status = mfrc522.MIFARE_Write(block, dataBlock, 16);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Writing failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
  } else {
    Serial.println("Data written successfully!");
  }

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();

  delay(2000);
}

