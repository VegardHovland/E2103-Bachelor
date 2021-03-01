#include <Wire.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define oled1 0x3C;
#define oled1 0x3D;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//Variables:
const byte slaveAddr[4]= {8, 9, 10 ,11};  // Defines the slavesadresses

const int numOfSlaves = 4;  //Defines number of slaves

int prevtime;
int x=0;
int encData[4]; //array to store encoder value from slaves


float angle[4];  //Array to sore the angles 0-360

void setup(){
  Wire.begin();         //Initialize wire

  
  Serial.begin(9600);   //starts serial monitor
  
  //OLED SETUP
   if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
   Serial.println(F("SSD1306 allocation failed"));
   for(;;);
 }
   display.setTextSize(2);
   display.setTextColor(WHITE);
   display.setCursor(0, 0);
   display.clearDisplay();
}

void loop() {
  
  getEncdata();
  //printText(angle);
  drawGraph(angle);

}


void getEncdata(){
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
  }
}

void printText(float ang[]){
  int currtime = millis();
  if( (currtime - prevtime) > 500){
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.clearDisplay();
    for(int i = 0; i < numOfSlaves; i++){
      display.print(i);                     // prints information in serial monitor
      display.print(": "); 
      display.println(ang[i]);
      display.display();
    }
    prevtime = millis();
  }
}

void drawGraph(float ang[]){
    
    if(x > 128){
      x = 0;
      display.clearDisplay();
    }
    
    for(int i = 0; i < numOfSlaves; i++){
     int y = map(ang[i], 0, 360, 0 , 64);
      display.drawPixel(x , y, WHITE);
      display.display();
    }
    x++;
 }
