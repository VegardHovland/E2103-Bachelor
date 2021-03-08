#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "actuator.h"
#include "DualG2HighPowerMotorShield.h"
#include "variables.h"


//Class declerations
Adafruit_SSD1306 display_1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);                                                                                // Declaration for display 1
Adafruit_SSD1306 display_2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);                                                                                // Declaration for display 1

DualG2HighPowerMotorShield24v14 md1(M11nSLEEP, M11DIR, M11PWM,  M11nFAULT,  M11CS, M12nSLEEP,  M12DIR,  M12PWM,  M12nFAULT, M12CS);                // Declaration for Motor driver 1
DualG2HighPowerMotorShield24v14 md2(M21nSLEEP, M21DIR, M21PWM,  M21nFAULT,  M21CS, M22nSLEEP,  M22DIR,  M22PWM,  M22nFAULT, M22CS);                // Declaration for Motor driver 2                                                     //defines the two motor drivers

Actuator actuators[4] = {Actuator(8), Actuator(9),Actuator(10), Actuator(11)};                                                                     // Generates a actuator list contaning 4 actuators and their i2c adress

//Declare functions-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void controllActuators(Actuator acts[]);         // Comutes the PID algorithm on every actuator
void printText(Actuator acts[]);                 // Displays data on oled 1
void drawGraph(Actuator acts[]);                 // Draw graph on oled 2
void setupOled();                                // Initialize oled displays
void stopIfFault();                              // Disable the motordrivers if there is a fault
void setupDrivers();                             // Start the motor drivers
void shutDown();                                 // Return to start and turn MOSFET off
void serialPrintData();                          // Print data for actuator 1-4
void updateSetpointSerial();                     // Update setpoint for a actuator using serial input
void updateParametersSerial();                   // Update parameters for a actuator using serial input
void printMenue();                               // Print menue for switch case
void serialPlot();                               // Print data to be represented in seial plot, PID tuning

void setup() {

    Serial.begin(9600);                          // Starts the serial monitor
    Wire.begin();                                // Initialize wire
    setupOled();                                
    setupDrivers();                              
    printMenue();                                
}

void loop() {
if (Serial.available() > 0) {
    char ch = Serial.read();                     // Gets user input if there is an input
    switch (ch) {               
      case 'a':{                                 // Updates the setpoint for a given actuator
        updateSetpointSerial();
        break;
      }
      case 'b': {                                // Updates the PID controller parameters for a given actuator using serial inputs
        updateParametersSerial();
        break;
      }
      case 'd': {                                // Print all data for actuators
         serialPrintData();
         break;
      }
      case 'p': {                                // Return to starposition and turn off
        printMenue();
        break;
      }
      case 'q': {                                // Return to starposition and turn off
        shutDown();
        break;
      }

      default : break;
    }
 }
      controllActuators(actuators);             // Pid controll on all the acuators by defaut
      drawGraph(actuators);                     // Draw graph at oled 1
      printText(actuators);                     // Print angles on oled 1
      serialPlot();                             // Print data to be plotted in serial plot
}


//Function defenitions--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Cpp file for gloable functions

//This prints data we want to plot in serial plot, in serial monitor choose serial plotter
void serialPlot(){                                      //We will tune the parameters using the first motor
   Serial.println(actuators[0].getSetpoint());          //print motor 1 setpoint
   Serial.println(actuators[0].getAngle());             //Print motor 1 angle
  // Serial.println(actuators[0].getSpeed());             //Print motor 1 speed
}

//Print menue for user inputs
void printMenue(){
    Serial.println("Press 'a' to update setpoint");
    Serial.println("Press 'b' to update PID parameters");
    Serial.println("Press 'd' to display information");
    Serial.println("Press 'p' to print menue");
    Serial.println("Press 'q' to exit");
}

//Function to update setpoint for a giver actuator using serial read
void updateSetpointSerial(){
        while(Serial.available()){int trash = Serial.read();}       //Clear trash from input buffer
        
        Serial.println("Skriv in nr på motor (1 - 4)");               // Ask for input
        while(!Serial.available()){};                               // Wait for input
        int i = Serial.parseInt() - 1;                              // Get act index (0-3)

        Serial.println("nytt setpunkt i grader");                   // Ask for input
        while(!Serial.available()){};                               // Wait for input
        double ang = Serial.parseFloat();                           // Get new setpoint
        
        actuators[i].setSetpoint(ang);                               // Update setpoint for giver actuator
        Serial.println("setpoint updated, press d to display data"); // confirm success
} 

//Function to update parameters for a giver actuator using serial read
void updateParametersSerial(){
        while(Serial.available()){int trash = Serial.read();}       //Clear trash from input buffer
          
        Serial.println("Skriv in nr på motor (1 - 4)");                // Ask for input
        while(!Serial.available()){};                                // Wait for input
        int i = Serial.parseInt() - 1;                               // Get act indx (1 - 3)
    
        Serial.println("kp");
        while(!Serial.available()){};
        float kp = Serial.parseFloat();                 // Get new kp
        
        Serial.println("ti");
        while(!Serial.available()){};
        float ti = Serial.parseFloat();                 // Get new Ti
        
        Serial.println("td");
        while(!Serial.available()){};
        float td = Serial.parseFloat();                 // Get new Td
        
        actuators[i].setParameters(kp, ti, td);    // Set new parameters for given actuator
        Serial.println("parameters updated, press d to display data"); // confirm success
}

//Controlls all the acuators
void controllActuators(Actuator actuators[]){
    for (int i = 0; i < numActuators; i++)                         // Loops over the 4 actuator objects    
    {     
        actuators[i].readAngle();                                  // Get the actuators angle
        actuators[i].computePID();                                 // Comeputes output using PID 
        if ( i == 0) {md1.setM2Speed(actuators[i].getSpeed());}    // Motor 1 is driver 1 M1
        if ( i == 1) {md1.setM2Speed(actuators[i].getSpeed());}    // Motor 2 is driver 1 M2   
        if ( i == 2) {md2.setM2Speed(actuators[i].getSpeed());}    // Motor 3 is driver 2 M1
        if ( i == 3) {md2.setM2Speed(actuators[i].getSpeed());}    // Motor 4 is driver 2 M2
    }
}


//Print text on oled 1
void printText(Actuator actuators[]){
  int currtime = millis();                                          // Store current time
  if( (currtime - printPrevtime) > 500){                            // Refresh display every .5 sek
    display_1.setTextSize(2);                                       // Chose text size 
    display_1.setTextColor(WHITE);                                  // Choose couler
    display_1.setCursor(0, 0);                                      // Return to upper left corner
    display_1.clearDisplay();                                       // Clear old data
    for(int i = 0; i < numActuators; i++){                          // Loop over act 1-4
      display_1.print(i+1);                                           // Prints information to display
      display_1.print(": "); 
      display_1.println(actuators[i].getAngle());
      display_1.display();                                          // Display on display 
    }
    printPrevtime = millis();                                       // Store previous refresh time
  }
}


//Draw graph on oled 2
void drawGraph(Actuator actuators[]){
    if(pixelX > 128){                                               // If at end of display
      pixelX = 0;                                                   // Reset x position
      display_2.clearDisplay();                                     // Clear display
    }
    
    for(int i = 0; i < numActuators; i++){                          // Loop over mact 1-4
     int y = map(actuators[i].getAngle(), 0, 360, 0 , 64);          // Get y coordinate
      display_2.drawPixel(pixelX , y, WHITE);                       // Print a pixel at x,y
      display_2.display();                                          // Display on display
    }
    pixelX++;                                                       // Increment x position
 }

 //Settup function for oled displays
 void setupOled(){
    //OLED SETUP for display 1
    if(!display_1.begin(SSD1306_SWITCHCAPVCC, oled1)){              // Start display and chack if sucess
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
    }
    
    display_1.setTextSize(2);                                       // Set display properties and clear display
    display_1.setTextColor(WHITE);
    display_1.setCursor(0, 0);
    display_1.clearDisplay();
    
    //OLED SETUP for display 2
    if(!display_2.begin(SSD1306_SWITCHCAPVCC, oled2)){              // Start display and chack if sucess
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
    }
    
    display_2.setTextSize(2);;                                       // Set display properties and clear display
    display_2.setTextColor(WHITE);
    display_2.setCursor(0, 0);
    display_2.clearDisplay();  
 }


 //Disable motordriver if fault
 void stopIfFault(){
  if (md1.getM1Fault() || md1.getM2Fault())                        // Checks if fault on driver 1
  {                   
    md1.disableDrivers();                                         // Disable driver 1
    delay(1);
    Serial.println("M1 fault");
    while (1);                                                    // Stop program
  } 
  if (md2.getM2Fault() || md2.getM1Fault() )                      // Checks if fault on driver 2
  {
    md2.disableDrivers();                                         // Disable driver 2
    delay(1);
    Serial.println("M2 fault");
    while (1);                                                    // Stop program                        
  }
 }

//Setup function for motor drivers
 void setupDrivers(){
    md1.init();                                                   // Init pinmodes driver 1
    md1.calibrateCurrentOffsets();
    md2.init();                                                   // Init pinmodes driver 2
    md2.calibrateCurrentOffsets();
    md1.enableDrivers();                                          // Enable mosfet 1
    md2.enableDrivers();                                          // Enable mosfet 2
    delay(50);                                                    // Enableing needs some time so delay

 }

 //return to start position and shutdown
 void shutDown(){
  Serial.println("Shutting down");
  int shutdowntime = millis() + 5000;                              //Generate a shutdown time
  while(shutdowntime > millis()){                                 // Comutes pid for actuators during shutdown time
      for (int i = 0; i < numActuators; i++){
        actuators[i].setSetpoint(startPos[i]);                    // Updates setpoints to startposition
      }
      
      for (int i = 0; i < numActuators; i++){                      // Loops over the 4 actuator objects     
        actuators[i].readAngle();                                  // Get the actuators angle
        actuators[i].computePID();                                 // Comeputes output using PID 
        if ( i == 0) {md1.setM2Speed(actuators[i].getSpeed());}    // Motor 1 is driver 1 M1
        if ( i == 1) {md1.setM2Speed(actuators[i].getSpeed());}    // Motor 2 is driver 1 M2
        if ( i == 2) {md2.setM2Speed(actuators[i].getSpeed());}    // Motor 3 is driver 2 M1
        if ( i == 3) {md2.setM2Speed(actuators[i].getSpeed());}    // Motor 2 is driver 2 M2
      }  
    }
    md1.setSpeeds(0, 0);                                           // Set speed to 0 for motor 1 and 2. NB! should already be 0
    md2.setSpeeds(0, 0);                                           // Set speed to 0 for motor 3 and 4. NB! should already be 0
    delay(50);                  
    md1.disableDrivers();                                          // Turn of mosfet 1
    md2.disableDrivers();                                          // Turn of mosfet 2
    Serial.print("shutdown complete");
 }

 void serialPrintData(){
    for(int i = 0; i < numActuators; i++){
      Serial.print(i+1);                                           //Print motor nr (1-4)
      Serial.println(" : "); 
       
      Serial.print("angle ");                                      // Prints joint angle in serial monitor
      Serial.print(": "); 
      Serial.println(actuators[i].getAngle());
      
      Serial.print("setpoint "); 
      Serial.print(i+1);                                           // Prints setpoint in serial monitor
      Serial.print(": "); 
      Serial.println(actuators[i].getSetpoint());
  
      Serial.print("Kp: ");                                        //Print parameters
      Serial.println(actuators[i].getKp()); 
      Serial.print("Ti: "); 
      Serial.println(actuators[i].getTi());
      Serial.print("Td: "); 
      Serial.println(actuators[i].getTd());
      
    }
    int amps [4];
    amps[0] = md1.getM1CurrentMilliamps();                         // Get current for motor 1
    amps[1] = md1.getM2CurrentMilliamps();                         // Get current for motor 2
    amps[2] = md2.getM1CurrentMilliamps();                         // Get current for motor 3
    amps[3] = md2.getM2CurrentMilliamps();                         // Get current for motor 4
    
    for(int i = 0; i < numActuators; i++){
      Serial.print("current "); 
      Serial.print(i+1);                                           // Prints the motors current i serial 
      Serial.print(": "); 
      Serial.println(amps[i]);
    }
 }
