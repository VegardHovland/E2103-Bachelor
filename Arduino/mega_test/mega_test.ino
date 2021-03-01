
#include <Wire.h>

#define slaveAddr 9

//Variables:
int angle;
//int angle[4]; //save analog value

void setup(){
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  Wire.beginTransmission(slaveAddr);
  int available = Wire.requestFrom(slaveAddr, (uint8_t)2);
  
  if(available == 2)
  {
    angle = Wire.read() << 8 | Wire.read(); 
    Serial.println(angle);
  }
  else
  {
    Serial.print("Unexpected number of bytes received: ");
    Serial.println(available);
  }

  int result = Wire.endTransmission();
  if(result)
  {
    Serial.print("Unexpected endTransmission result: ");
    Serial.println(result);
  }
}
