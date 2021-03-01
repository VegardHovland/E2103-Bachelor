
#include <Wire.h>

#define slaveAddr 9

//Constants:
const int potPin1 = A1; //pin A0 to read analog input
const int potPin2 = A2; //pin A0 to read analog input
const int potPin3 = A3; //pin A0 to read analog input
const int potPin4 = A4; //pin A0 to read analog input

//Variables:

int angle[4]; //save analog value

void setup(){
  //Input or output? 
  Serial.begin(9600);
  Wire.begin();
 
}

void loop(){

  for(int i=0 ; i < 4; i++){
    Serial.println(angle[i]);  
  }

  Wire.beginTransmission(slaveAddr);
  Wire.write(angle);
  Wire.endTransmission;
 

  delay(100);                          //Small delay
}
