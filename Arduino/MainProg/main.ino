//Hovedprogrammet, her skal minst mulig kode skrives og brukes est mulig funksjoner!
//Regler vi bruker upperLowerCase (camelCase)
//HUSK Å KOMMENTERE!!!

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "actuator.h"

//Variables
//Defines
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define oled1 0x3C
#define oled2 0x3D

//Variables---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
const int numActuators = 4 ; // Defines the number of actuators
int printPrevtime;
int pixelX=0;    //Stores the x value for pixel drwing for oled

//Defines different classes
Adafruit_SSD1306 display_1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_SSD1306 display_2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Actuator actuators[4] = {Actuator(8 , 3), Actuator(9 ,5),Actuator(10 ,6), Actuator(11 ,8) };     //Generates a actuator list contaning 4 actuators and their i2c adress and outputpin

//Declare functions
void controllActuators(Actuator acts[]);    //comutes the PID algorithm on every actuator
void printText(Actuator acts[]);                //Displays data on oled 1
void drawGraph(Actuator acts[]);                //Draw graph on oled 2
void setupOled();                          //initialize oled displays

void setup() {

    Serial.begin(9600);                                                                       //Starts the serial monitor
    Wire.begin();                                                                            //Initialize wire
    setupOled();                                                                              //Initialize oled displays
}

void loop() {
if (Serial.available() > 0) {
    char ch = Serial.read();                                                                  //Gets user input
    switch (ch) {               
      case 'a':{                                                                             //Updates the setpoint for a given actuator
        Serial.print("Skriv in nr på motor");
        int i = Serial.read();
        Serial.print("nytt setpunkt i grader");
        double ang = Serial.read();
        actuators[i].setSetpoint(ang);
        break;
      }
      case 'b': {                                                                             //Updates the PID controller parameters for a given actuator
        Serial.print("Skriv in nr på motor");
        int i = Serial.read();
        Serial.print("kp");
        double kp = Serial.read();
        Serial.print("ti");
        double ti = Serial.read();
        Serial.print("td");
        double td = Serial.read();
        actuators[i].setParameters(kp, ti, td);
        break;
      }
      case 'q': break;
        //returner til start konfigurasjon

      default :
      {
        controllActuators(actuators);                                                           //Pid controll on all the acuators by defaut
        drawGraph(actuators);
        printText(actuators);
        break;
      }
    }
    }
        controllActuators(actuators);                                                           //Pid controll on all the acuators by defaut
        drawGraph(actuators);
        printText(actuators);
}


//Function defenitions
//Cpp file for gloable functions

#include "actuator.h"

//Controlls all the acuators
void controllActuators(Actuator actuators[]){
    for (int i = 0; i < numActuators; i++)              // Loops over the 4 actuator objects    
    {
        actuators[i].readAngle();          //Get the actuators angle
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
    for(int i = 0; i < numActuators; i++){
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
    
    for(int i = 0; i < numActuators; i++){
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
