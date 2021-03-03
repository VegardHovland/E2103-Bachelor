#include <Wire.h>

#define slaveAddr 9

//Constants:
const int potPin = A0; //pin A0 to read analog input


//Variables:
int input; //save analog value



void setup(){
  //Define input mode

  Serial.begin(9600);
  Wire.begin(slaveAddr);
  Wire.onRequest(requestEvent); // register event
  
  pinMode(potPin, INPUT); //Optional 
  
}

void loop(){
  
}

void requestEvent() {
  input =  analogRead(potPin);          //Read and save analog value from potentiometer
  uint8_t buffer[2];

  buffer[0] = input >> 8;
  buffer[1] = input & 0xff;
  
  
  Wire.write(buffer,2); // respond with message of 6 bytes
  
}
