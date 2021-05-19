#include <Wire.h>
#include <EEPROM.h>           // Liberary for EEPROM saving

#define slaveAddr 10
//Constants:
int pinA = 3;                 // Encoder pin for A puls
int pinB = 4;                 // Encoder pin for B puls

//Variables:
int counter  = -4480 / 4;     // Store the incremental encoders counter, slave 3 starts at 90 deg
int aState;                   // Store the state of the puls
int aLastState;               // Save last state of the puls
void setup() {
  pinMode (pinA, INPUT);      //Defines the input pins
  pinMode (pinB, INPUT);

  Wire.begin(slaveAddr);
  Wire.onRequest(requestEvent);              // On request from master function
  aLastState = digitalRead(pinA);            // Reads the initial state of the outputA
  attachInterrupt(digitalPinToInterrupt(2), saveToERPROM, FALLING);  //Atatches interupt pin
  EEPROM.get(0, counter);                                            //Get last stored counter value
}

void loop() {
  aState = digitalRead(pinA);                // State of puls A
  if (aState != aLastState) {                // Checks if a pulse has happened
    if (digitalRead(pinB) != aState) {       // checks if clockwise rotation
      counter ++;
    } else {
      counter --;
    }
  }
  aLastState = aState;                       // Saves previous state
}

void requestEvent() {
  uint8_t buffer[2];

  buffer[0] = counter >> 8;                     // Store the int as 6 bytes
  buffer[1] = counter & 0xff;

  Wire.write(buffer, 2);                        // Respond with message of 6 bytes
}
void saveToERPROM(){                            // ISR function for interupt
    EEPROM.put(0, counter);                     // Store counter value
    delay(100);                                 // Wait to die
}
