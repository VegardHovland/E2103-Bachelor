#include <Wire.h>

//Constants:
int pinA = 2;                 //Encoder pin for A puls
int pinB = 3;                 //Encoder pin for B puls

//Variables:  
int counter = 0;            //store the incremental encoders 
int aState;                 //Store the state of the puls
int aLastState;             //Save last state of the puls
void setup(){
  pinMode (pinA,INPUT);                     //Defines the input pins
  pinMode (pinB,INPUT);

  Serial.begin(9600);
  Wire.begin(slaveAddr);
  Wire.onRequest(requestEvent); // register event
  
  aLastState = digitalRead(pinA);            // Reads the initial state of the outputA
  
}

void loop(){
   aState = digitalRead(pinA);                // state of puls A
   if (aState != aLastState){                 // Checks if a pulse has happened
     if (digitalRead(pinB) != aState) {       // checks if clockwise rotation
       counter ++;
     } else {
       counter --;
     }
     Serial.print("Position: ");              //Prints the postion 
     Serial.println(counter);
   } 
   aLastState = aState;                      // Saves previous state
}

void requestEvent() {
  uint8_t buffer[2];

  buffer[0] = counter >> 8;
  buffer[1] = counter & 0xff;
  
  Wire.write(buffer,2);             // respond with message of 6 bytes
  
}
