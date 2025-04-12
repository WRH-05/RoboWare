/*
ESP32 Robot Navigation with Dijkstra Path Planning & Elevator Activation

This sketch demonstrates:
• Storing a map as an adjacency matrix (with estimated distances)
• Using Dijkstra’s algorithm to compute the best path from HOME to a TAG node
• Following a line segment using QTR sensors
• At intersections (nodes on the route) activating a “scissor elevator” for demonstration

Modify the graph distances, sensor/motor pins, PWM values, movement routines,
and elevator logic to match your robot’s hardware and environment.
*/

#include <Arduino.h>
#include <QTRSensors.h>

//────────────────────────────
// 1. SENSOR SETUP (Line Following)
//
const uint8_t sensorPins[8] = {34, 35, 32, 33, 25, 26, 27, 14};
QTRSensors qtr;
const uint8_t numSensors = 8;
uint16_t sensorValues[numSensors];
const uint16_t lineThreshold = 3500; // Adjust based on calibration

//────────────────────────────
// 2. MOTOR SETUP
//
const int rightIn1 = 12;
const int rightIn2 = 13;
const int rightPWM = 15; // PWM pin for right motor

const int leftIn1 = 16;
const int leftIn2 = 17;
const int leftPWM = 18; // PWM pin for left motor

// LEDC (PWM) settings for ESP32
const int pwmFreq = 5000;
const int pwmResolution = 8; // 8-bit (0-255)
const int rightChannel = 0;
const int leftChannel = 1;
const int baseSpeed = 150; // Adjust (0-255)
const int maxSpeed = 255;

//────────────────────────────
// 3. MAP & DIJKSTRA PATH PLANNING
//
#define NUM_NODES 5
#define INF 1000000 // large value for no connection

enum Node {
  HOME = 0,
  TAG1,
  TAG2,
  TAG3,
  TAG4
};

// Adjacency matrix for nodes
int graph[NUM_NODES][NUM_NODES] = {
  // HOME, TAG1, TAG2, TAG3, TAG4
  /* HOME */ { 0,   120, 50,  INF, 45 },
  /* TAG1 */ { 120, 0,   70,  60,  INF },
  /* TAG2 */ { 50,  70,  0,   80,  100 },
  /* TAG3 */ { INF, 60,  80,  0,   90 },
  /* TAG4 */ { 45,  INF, 100, 90,  0 }
};

int distanceFromStart[NUM_NODES];
int previous[NUM_NODES];
bool visited[NUM_NODES];

// Dijkstra's algorithm using an adjacency matrix.
void dijkstra(int start) {
  for (int i = 0; i < NUM_NODES; i++) {
    distanceFromStart[i] = INF;
    visited[i] = false;
    previous[i] = -1;
  }
  distanceFromStart[start] = 0;
  
  for (int i = 0; i < NUM_NODES; i++) {
    int minDist = INF, u = -1;
    for (int j = 0; j < NUM_NODES; j++) {
      if (!visited[j] && distanceFromStart[j] < minDist) {
        minDist = distanceFromStart[j];
        u = j;
      }
    }
    if (u == -1) break;
    visited[u] = true;
    
    for (int v = 0; v < NUM_NODES; v++) {
      if (!visited[v] && graph[u][v] < INF) {
        int alt = distanceFromStart[u] + graph[u][v];
        if (alt < distanceFromStart[v]) {
          distanceFromStart[v] = alt;
          previous[v] = u;
        }
      }
    }
  }
}

// Reconstruct path (in reverse) from start to destination.
void getPath(int start, int destination, int path[], int &pathLength) {
  pathLength = 0;
  int current = destination;
  while (current != -1) {
    path[pathLength++] = current;
    if (current == start) break;
    current = previous[current];
  }
  // Reverse the path array
  for (int i = 0; i < pathLength/2; i++) {
    int temp = path[i];
    path[i] = path[pathLength - 1 - i];
    path[pathLength - 1 - i] = temp;
  }
}

void printPath(int path[], int length) {
  for (int i = 0; i < length; i++) {
    String name;
    switch (path[i]) {
      case HOME: name = "HOME"; break;
      case TAG1: name = "TAG1"; break;
      case TAG2: name = "TAG2"; break;
      case TAG3: name = "TAG3"; break;
      case TAG4: name = "TAG4"; break;
      default: name = "Unknown";
    }
    Serial.print(name);
    if (i < length - 1) Serial.print(" -> ");
  }
  Serial.println();
}

//────────────────────────────
// 4. MOVEMENT FUNCTIONS
//
void setMotorSpeed(int in1, int in2, int pwmPin, int speed, int ledcChannel) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  ledcWrite(ledcChannel, speed);
}

void lineFollowSegment() {
  // Read sensor array and update sensorValues
  uint16_t position = qtr.readLineWhite(sensorValues);
  int error = (int)position - lineThreshold;
  float Kp = 0.1; // Proportional gain; adjust as needed
  int correction = (int)(Kp * error);
  
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);
  
  setMotorSpeed(leftIn1, leftIn2, leftPWM, leftSpeed, leftChannel);
  setMotorSpeed(rightIn1, rightIn2, rightPWM, rightSpeed, rightChannel);
}

// A simple routine to execute a 90° turn.
// direction: 1 for right, -1 for left.
void executeTurn(int direction) {
  // Stop before turn.
  setMotorSpeed(leftIn1, leftIn2, leftPWM, 0, leftChannel);
  setMotorSpeed(rightIn1, rightIn2, rightPWM, 0, rightChannel);
  delay(150);
  
  int turnSpeed = baseSpeed;
  int turnDuration = 400; // milliseconds (adjust as needed)
  
  if(direction == 1) { // right turn: right motor backward, left motor forward
    digitalWrite(rightIn1, LOW);
    digitalWrite(rightIn2, HIGH);
    digitalWrite(leftIn1, HIGH);
    digitalWrite(leftIn2, LOW);
  } else if(direction == -1) { // left turn: right motor forward, left motor backward
    digitalWrite(rightIn1, HIGH);
    digitalWrite(rightIn2, LOW);
    digitalWrite(leftIn1, LOW);
    digitalWrite(leftIn2, HIGH);
  }
  
  ledcWrite(leftChannel, turnSpeed);
  ledcWrite(rightChannel, turnSpeed);
  delay(turnDuration);
  
  setMotorSpeed(leftIn1, leftIn2, leftPWM, 0, leftChannel);
  setMotorSpeed(rightIn1, rightIn2, rightPWM, 0, rightChannel);
  delay(150);
}

//────────────────────────────
// 5. ELEVATOR FUNCTION
//
// This function simulates elevating a scissor mechanism.
// Adjust the pin and timing as per your actual hardware.
const int elevatorPin = 4; // Example pin for the elevator actuator

void activateElevator() {
  Serial.println("Elevator activated!");
  digitalWrite(elevatorPin, HIGH);  // Activate elevator (e.g., turn on motor)
  delay(2000);                      // Keep elevator activated for 2 seconds
  digitalWrite(elevatorPin, LOW);   // Deactivate elevator
}

//────────────────────────────
// 6. INTERSECTION DETECTION
//
// A simple routine to detect an intersection from sensor readings.
// In this example, if most sensors are above the threshold, we assume it’s an intersection.
bool intersectionDetected() {
  int count = 0;
  for (int i = 0; i < numSensors; i++) {
    if(sensorValues[i] > lineThreshold) count++;
  }
  // If almost all sensors detect the line (e.g., 6 or more of 8), assume an intersection.
  return (count >= 6);
}

//────────────────────────────
// 7. SETUP & MAIN PROGRAM
//
int route[NUM_NODES];    // Stores the computed path (nodes)
int routeLength = 0;     // Length of the path
int currentRouteIndex = 0; // Current node in the route we're moving toward

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Starting ESP32 Robot Navigation Demo...");

  // Initialize motor control pins.
  pinMode(rightIn1, OUTPUT);
  pinMode(rightIn2, OUTPUT);
  pinMode(leftIn1, OUTPUT);
  pinMode(leftIn2, OUTPUT);
  
  // Setup LEDC PWM channels.
  ledcSetup(rightChannel, pwmFreq, pwmResolution);
  ledcAttachPin(rightPWM, rightChannel);
  
  ledcSetup(leftChannel, pwmFreq, pwmResolution);
  ledcAttachPin(leftPWM, leftChannel);
  
  // Initialize QTR sensor array.
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, numSensors);
  delay(500);
  
  Serial.println("Calibrating sensors...");
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(20);
  }
  Serial.println("Calibration complete.");
  
  // Set up elevator actuator pin.
  pinMode(elevatorPin, OUTPUT);
  digitalWrite(elevatorPin, LOW);
  
  //────────────────────────────
  // Compute the optimal route using Dijkstra's algorithm.
  int destination = TAG4; // Change as needed (e.g., TAG1, TAG2, etc.)
  dijkstra(HOME);
  getPath(HOME, destination, route, routeLength);
  Serial.print("Optimal path from HOME to destination: ");
  printPath(route, routeLength);
  
  // Set our current route index (start at HOME).
  currentRouteIndex = 0;
}

void loop() {
  // Continually follow the line.
  lineFollowSegment();
  
  // Read sensor values continuously (they are updated in lineFollowSegment)
  
  // Check for an intersection (e.g., at a designated waypoint).
  if(intersectionDetected()) {
    Serial.println("Intersection detected!");
    // Advance to the next node on the planned route if available.
    if (currentRouteIndex < routeLength - 1) {
      currentRouteIndex++;
      // If the new node is not HOME (i.e., it’s a TAG node), activate the elevator.
      if(route[currentRouteIndex] != HOME) {
        Serial.print("Reached node ");
        switch (route[currentRouteIndex]) {
          case TAG1: Serial.println("TAG1"); break;
          case TAG2: Serial.println("TAG2"); break;
          case TAG3: Serial.println("TAG3"); break;
          case TAG4: Serial.println("TAG4"); break;
          default: Serial.println("Unknown");
        }
        // For demo: elevate at this designated tag.
        activateElevator();
      }
      // Execute a turn (for demonstration, assume a right turn; in practice, you would choose direction based on the path).
      executeTurn(1);
    }
    // Small delay to avoid multiple detections of the same intersection.
    delay(1000);
  }
  
  delay(50); // Loop delay for stability.
}
