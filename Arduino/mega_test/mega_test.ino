
#include <Wire.h>
const byte slaveAddr[4]= {8, 9, 10 ,11};  // Defines the slavesadresses

//Variables:
const int numOfSlaves = 4;  //Defines number of slaves


int encData[4]; //array to store encoder value from slaves
float angle[4];  //Array to sore the angles 0-360

void setup(){
  Wire.begin();         //Initialize wire
  Serial.begin(9600);   //starts serial monitor
}

void loop() {
  

  for(int i = 0; i < numOfSlaves; i++){     //Loops over the 4 slaves and asks for their data
    Wire.beginTransmission(slaveAddr[i]);   //starts transmition with slaver nr. i
    int available = Wire.requestFrom(slaveAddr[i], (uint8_t)2);  //requests bytes from slave
  
    if(available == 2)                //Checks if 2 bytes are avavible
    {
      encData[i] = Wire.read() << 8 | Wire.read();       //Reads upper and lower byte and converts to int
    }
    else                                                  //Error in transmition
    {
      Serial.print("Unexpected number of bytes received: ");
      Serial.println(available);
    }

    int result = Wire.endTransmission();                //end transmition and chack if sucessfulll
    if(result)
    {
      Serial.print("Unexpected endTransmission result: ");
      Serial.println(result);
    }
     angle[i] = (float(encData[i])*360.0)/1023.0;     //Converts to degrees (0-360)
     Serial.print("slaveAddr ");                     // prints information in serial monitor
     Serial.println(slaveAddr[i]);
      Serial.print("ang: ");
     Serial.println(angle[i], 6);
  }
}
