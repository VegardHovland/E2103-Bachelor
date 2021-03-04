#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "actuator.h"
#include "DualG2HighPowerMotorShield.h"
#include "variables.h"


//Defines different classes
Adafruit_SSD1306 display_1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);                             //defines oled display 1
Adafruit_SSD1306 display_2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);                             //defines oled 2
DualG2HighPowerMotorShield24v14 md1(M11nSLEEP, M11DIR, M11PWM,  M11nFAULT,  M11CS, M12nSLEEP,  M12DIR,  M12PWM,  M12nFAULT, M12CS);
DualG2HighPowerMotorShield24v14 md2(M21nSLEEP, M21DIR, M21PWM,  M21nFAULT,  M21CS, M22nSLEEP,  M22DIR,  M22PWM,  M22nFAULT, M22CS);                                                       //defines the two motor drivers
Actuator actuators[4] = {Actuator(8 , 3), Actuator(9 ,5),Actuator(10 ,6), Actuator(11 ,8) };     //Generates a actuator list contaning 4 actuators and their i2c adress and outputpin

//Declare functions-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void controllActuators(Actuator acts[]);    //comutes the PID algorithm on every actuator
void printText(Actuator acts[]);                //Displays data on oled 1
void drawGraph(Actuator acts[]);                //Draw graph on oled 2
void setupOled();                          //initialize oled displays
void stopIfFault();                        //disable the motordrivers if there is a fault
void setupDrivers();
void shutDown();
void serialPrintData();

void setup() {

    Serial.begin(9600);                                                                      //Starts the serial monitor
    Wire.begin();                                                                            //Initialize wire
    setupOled();                                                                             //Initialize oled displays
    setupDrivers();
    
}

void loop() {
if (Serial.available() > 0) {
    char ch = Serial.read();                                                                 //Gets user input
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
      case 'd': {
         serialPrintData();
         break;
      }
      case 'q': {
        shutDown();
        break;
      }

      default : break;
    }
 }
      controllActuators(actuators);                                                           //Pid controll on all the acuators by defaut
      drawGraph(actuators);
      printText(actuators);
}


//Function defenitions--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Cpp file for gloable functions

//Controlls all the acuators
void controllActuators(Actuator actuators[]){
    for (int i = 0; i < numActuators; i++)              // Loops over the 4 actuator objects    
    {     
        actuators[i].readAngle();          //Get the actuators angle
        actuators[i].computePID();                      //comeputes output using PID 
        if ( i == 0) {md1.setM2Speed(actuators[i].getSpeed());}
        if ( i == 1) {md1.setM2Speed(actuators[i].getSpeed());}
        if ( i == 2) {md2.setM2Speed(actuators[i].getSpeed());}
        if ( i == 3) {md2.setM2Speed(actuators[i].getSpeed());}
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


 //Disable motordriver if fault
 void stopIfFault(){
  if (md1.getM1Fault() || md1.getM2Fault())   //Checks if fault on driver 1
  {                   
    md1.disableDrivers();                     //Disable driver 1
    delay(1);
    Serial.println("M1 fault");
    while (1);
  }
  if (md2.getM2Fault() || md2.getM1Fault() )  //Checks if fault on driver 2
  {
    md2.disableDrivers();                     //Disable driver 2
    delay(1);
    Serial.println("M2 fault");
    while (1);
  }
 }

 void setupDrivers(){
    md1.init();
    md1.calibrateCurrentOffsets();
    md2.init();
    md2.calibrateCurrentOffsets();
    md1.enableDrivers();
    md2.enableDrivers();
    delay(50);

 }

 void shutDown(){
  //return to start position
  md1.setSpeeds(0, 0);
  md2.setSpeeds(0, 0);
  delay(50);
  md1.disableDrivers();
  md2.disableDrivers();
  
 }

 void serialPrintData(){
    for(int i = 0; i < numActuators; i++){
      Serial.print("angle "); 
      Serial.print(i);                     // prints joint angle in serial monitor
      Serial.print(": "); 
      Serial.println(actuators[i].getAngle());
      
      Serial.print("setpoint "); 
      Serial.print(i);                     // prints setpoint in serial monitor
      Serial.print(": "); 
      Serial.println(actuators[i].getSetpoint());
  }
    int amps [4];
    amps[0] = md1.getM1CurrentMilliamps();
    amps[1] = md1.getM2CurrentMilliamps();
    amps[2] = md2.getM1CurrentMilliamps();
    amps[3] = md2.getM2CurrentMilliamps();
    
    for(int i = 0; i < numActuators; i++){
      Serial.print("current "); 
      Serial.print(i);                     // prints joint angle in serial monitor
      Serial.print(": "); 
      Serial.println(amps[i]);
    }
 }
