#include <Wire.h>

#define slaveAddr 9

//Constants:
const int potPin1 = A1; //pin A0 to read analog input
const int potPin2 = A2; //pin A0 to read analog input
const int potPin3 = A3; //pin A0 to read analog input
const int potPin4 = A4; //pin A0 to read analog input

const int comPin1 = 6; //pin A0 to read analog input
const int comPin2 = 9; //pin A0 to read analog input
const int comPin3 = 10; //pin A0 to read analog input
const int comPin4 = 11; //pin A0 to read analog input


//Variables:
int value1; //save analog value
int value2; //save analog value1
int value3; //save analog value
int value4; //save analog value
int angle[4];


void setup(){
  //Define input mode

  Serial.begin(9600);
  Wire.begin(slaveAddr);
  
  pinMode(potPin1, INPUT); //Optional 
  pinMode(potPin2, INPUT); //Optional 
  pinMode(potPin3, INPUT); //Optional 
  pinMode(potPin4, INPUT); //Optional 

  
}

void loop(){
  value1 =  analogRead(potPin1);          //Read and save analog value from potentiometer
  value1 =  map(value1, 0, 1023, 0, 255); //Map value 0-1023 to 0-255 (PWM)

  value2 =  analogRead(potPin2);           //Read and save analog value from potentiometer
  value2 =  map(value2, 0, 1023, 0, 255); //Map value 0-1023 to 0-255 (PWM)

  value3 =  analogRead(potPin3);           //Read and save analog value from potentiometer
  value3 =  map(value3, 0, 1023, 0, 255); //Map value 0-1023 to 0-255 (PWM)

  value4 =  analogRead(potPin4);           //Read and save analog value from potentiometer
  value4 =  map(value4, 0, 1023, 0, 255); //Map value 0-1023 to 0-255 (PWM)

  angle=[value1, value2, value3,value4];
  
  Serial.println(value1);
  
  delay(100);                             //Small delay
}
