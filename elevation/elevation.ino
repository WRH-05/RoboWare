/*
 * Arduino Uno code to continuously alternate the rotation
 * of a bipolar stepper motor using a L298N driver.
 *
 * The motor rotates in one direction for 2 seconds and then
 * rotates in the opposite direction for 2 seconds, repeating indefinitely.
 *
 * Wiring connections (adjust as necessary):
 *   - L298N IN1  <---> Arduino digital pin 8
 *   - L298N IN2  <---> Arduino digital pin 9
 *   - L298N IN3  <---> Arduino digital pin 10
 *   - L298N IN4  <---> Arduino digital pin 11
 */

// Define Arduino Uno pins connected to the L298N driver.
const int motorControlPin1 = 8;  
const int motorControlPin2 = 9;
const int motorControlPin3 = 10;
const int motorControlPin4 = 11;

// Full-step sequence for driving the bipolar stepper motor.
int stepSequence[4][4] = {
  {1, 0, 1, 0},
  {0, 1, 1, 0},
  {0, 1, 0, 1},
  {1, 0, 0, 1}
};

const int totalStepsInSequence = 4;

// Fast step delay in milliseconds.
const int stepDelay = 2;

// Function that sends the proper signals to the motor driver for a given step.
void stepMotor(int sequenceIndex) {
  digitalWrite(motorControlPin1, stepSequence[sequenceIndex][0]);
  digitalWrite(motorControlPin2, stepSequence[sequenceIndex][1]);
  digitalWrite(motorControlPin3, stepSequence[sequenceIndex][2]);
  digitalWrite(motorControlPin4, stepSequence[sequenceIndex][3]);
}

// Rotate the stepper motor for a given duration (milliseconds).
// 'direction' = 1 for forward; -1 for reverse.
void rotateForDuration(int direction, unsigned long durationMs) {
  unsigned long startTime = millis();
  int sequenceIndex = 0;

  while (millis() - startTime < durationMs) {
    // Activate the appropriate step in the sequence.
    stepMotor(sequenceIndex);
    delay(stepDelay);

    // Update the sequence index based on the desired direction.
    if (direction > 0) {
      sequenceIndex = (sequenceIndex + 1) % totalStepsInSequence;
    } else {
      // Ensure proper wrapping when decrementing.
      sequenceIndex = (sequenceIndex - 1 + totalStepsInSequence) % totalStepsInSequence;
    }
  }
  // Optionally, de-energize the motor coils after rotation.
  digitalWrite(motorControlPin1, LOW);
  digitalWrite(motorControlPin2, LOW);
  digitalWrite(motorControlPin3, LOW);
  digitalWrite(motorControlPin4, LOW);
}

void setup() {
  // Initialize serial communication for debugging (optional)
  Serial.begin(9600);

  // Initialize control pins as outputs.
  pinMode(motorControlPin1, OUTPUT);
  pinMode(motorControlPin2, OUTPUT);
  pinMode(motorControlPin3, OUTPUT);
  pinMode(motorControlPin4, OUTPUT);
}

void loop() {
  // Rotate in one direction for 2 seconds.
  rotateForDuration(1, 2000);
  
  // Rotate in the opposite direction for 2 seconds.
  rotateForDuration(-1, 2000);
}
    