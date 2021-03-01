#include <Wire.h>

#define slaveAddr 9

//Constants:
const int potPin = A0; //pin A0 to read analog input


//Variables:
float value1; //save analog value



void setup(){
  //Define input mode

  Serial.begin(9600);
  Wire.begin(slaveAddr);
  Wire.onRequest(requestEvent); // register event
  
  pinMode(potPin, INPUT); //Optional 
  
}

void loop(){
  value1 =  analogRead(potPin);          //Read and save analog value from potentiometer
 
  
  Serial.println(value1);
  
  delay(100);                             //Small delay
}

void requestEvent() {
  Wire.write("hello "); // respond with message of 6 bytes
  // as expected by master
}
