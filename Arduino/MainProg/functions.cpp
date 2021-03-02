//Cpp file for gloable functions
#include "functions.h"
#include "constants.h"
#include "actuator.h"

//Controlls all the acuators
void controllActuators(Actuator actuators[]){
    for (int i = 0; i < numActuators; i++)              // Loops over the 4 actuator objects    
    {
        actuators[i].readAngle(i,slaveAddr[i]);          //Get the actuators angle
        actuators[i].computePID();                      //comeputes output using PID 
        actuators[i].setOutput();                       //sets out the putput to the actuator
    }
}


//Print text on oled 1
void printText(Actuator actuators[]){
  int currtime = millis();
  if( (currtime - printPrevtime) > 500){
    display_1.setTextSize(2);
    display_1.setTextColor(WHITE);
    display_1.setCursor(0, 0);
    display_1.clearDisplay();
    for(int i = 0; i < numOfSlaves; i++){
      display_1.print(i);                     // prints information in serial monitor
      display_1.print(": "); 
      display_1.println(actuators[i].getAngle());
      display_1.display();
    }
    printPrevtime = millis();
  }
}


//Draw graph on oled 2
void drawGraph(Actuator actuators[]){
    
    if(pixelX > 128){
      pixelX = 0;
      display_2.clearDisplay();
    }
    
    for(int i = 0; i < numOfSlaves; i++){
     int y = map(actuators[i].getAngle(), 0, 360, 0 , 64);
      display_2.drawPixel(pixelX , y, WHITE);
      display_2.display();
    }
    pixelX++;
 }

 //Settup function for oled displays
 void setupOled(){
    //OLED SETUP for display 1
    if(!display_1.begin(SSD1306_SWITCHCAPVCC, oled1)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
    }
    display_1.setTextSize(2);
    display_1.setTextColor(WHITE);
    display_1.setCursor(0, 0);
    display_1.clearDisplay();
    
    //OLED SETUP for display 2
    if(!display_2.begin(SSD1306_SWITCHCAPVCC, oled2)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
    }
    display_2.setTextSize(2);
    display_2.setTextColor(WHITE);
    display_2.setCursor(0, 0);
    display_2.clearDisplay();  
 }