#define BLYNK_TEMPLATE_ID "TMPL6_faDFMsB"
#define BLYNK_TEMPLATE_NAME "Vertical Carousel"

#define BLYNK_FIRMWARE_VERSION "0.1.2"
#define BLYNK_PRINT Serial

//#define BLYNK_DEBUG

#define APP_DEBUG

// Uncomment your board, or configure a custom board in Settings.h
#define USE_ESP32_DEV_MODULE

#include "BlynkEdgent.h"
#include <HardwareSerial.h>

// Initialize hardware serial (use UART2 for flexibility)
HardwareSerial nextionSerial(2);

// Define the RX and TX pins for the ESP32 (match this to your wiring)
#define NEXTION_RX_PIN 16
#define NEXTION_TX_PIN 17

// Pin definitions
const int stepPin = 25; // Connect to the step pin of the stepper driver
const int dirPin = 26;  // Connect to the direction pin of the stepper driver

// Motor parameters
const int stepsPerRevolution = 200;      // Steps in one revolution
const float rotationsPerPosition = 47.62; // Rotations per position change
const int totalPositions = 12;          // Total compartments (0-12)

// Variable to track the current position
int currentPosition = 1;



int Input_Value = 0;
int Current_state = 1;

bool aboveThreshold = false; 


void setup()
{
 // Start serial communication for debugging
 Serial.begin(115200);

 // Start serial communication with the Nextion
 nextionSerial.begin(9600, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
 pinMode(stepPin, OUTPUT); // Set step pin as output
 pinMode(dirPin, OUTPUT);  // Set direction pin as output
 
  BlynkEdgent.begin();
    Blynk.virtualWrite(V0,0);
 Blynk.virtualWrite(V1,0);
  Blynk.virtualWrite(V2,0);

}

void loop() {
  BlynkEdgent.run();
  //Serial.println("NOW");
  
    // Check if data is available from the Nextion
    
  if (nextionSerial.available() > 0) {
    // Read available data into a buffer
    String receivedData = "";
    while (nextionSerial.available()) {
      char c = nextionSerial.read();
      receivedData += c;
      delay(2); // Small delay to ensure all bytes are read
    }

    // Display the received data on the serial monitor
    Serial.print("Received Data: ");
    Serial.println(receivedData);

    // Convert received ASCII bytes to a number
    int number = receivedData.toInt(); // Convert string to integer

    // Execute rotations if the number is valid
    if (number >= 1 && number <= 12) { // Define valid range
      moveToPosition(number);
     //Serial.print("Rotating Motor for ");
     // Serial.print(number);
     // Serial.println(" complete rotations.");
      
     // rotateMotor(number * 2); // Perform rotations
    } else {
      Serial.println("Incorrect Data");
    }
  }

 
}



BLYNK_WRITE(V0) {
 Input_Value = param.asInt();
 Serial.print("Comapartment number entered from Blynk: ");
 Serial.println(Input_Value);
  moveToPosition(Input_Value);
  
}


// Function to move the motor to a specific position
void moveToPosition(int newPosition) {
  if (newPosition < 1 || newPosition > totalPositions) {
    Serial.println("Error: Invalid compartment number!");
    return;
  }

  int positionsToMove = (newPosition - currentPosition + totalPositions) % totalPositions;
  bool direction = positionsToMove <= totalPositions / 2; // Clockwise or counterclockwise
  
  if (!direction) {
    positionsToMove = totalPositions - positionsToMove; // Reverse movement for shorter path
  }
     Blynk.virtualWrite(V2,1);
  for (int i = 0; i < positionsToMove; i++) {
    moveOnePosition(direction);
    
  }
    Blynk.virtualWrite(V2,0);
  currentPosition = newPosition; // Update current position
  Serial.println("Desired Position Reached");
  Blynk.virtualWrite(V1,currentPosition);
}



// Function to move the motor one position
void moveOnePosition(bool direction) {
  int stepsToMove = stepsPerRevolution * rotationsPerPosition; // Steps for one position
  Serial.println("Moving Motor");
  
  digitalWrite(dirPin, direction); // Set the direction of rotation
  
  for (int i = 0; i < stepsToMove; i++) {
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(50); // Adjust this for motor speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(50); // Adjust this for motor speed
  }
}
