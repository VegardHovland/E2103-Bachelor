#ifndef constants_h
#define constants_h

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "actuator.h"



#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define oled1 0x3C
#define oled2 0x3D

Adafruit_SSD1306 display_1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_SSD1306 display_2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

Actuator actuators[] = {Actuator(2 , 3), Actuator(4 ,5),Actuator(5 ,6), Actuator(7 ,8) };     //Generates a actuator list contaning 4 actuators


//Header file for global variables

const int numActuators = 4 ; // Defines the number of actuators
const int numOfSlaves = 4;  //Defines number of slaves

const byte slaveAddr[4]= {8, 9, 10 ,11};  // Defines the slavesadresses


int printPrevtime;
int pixelX=0;    //Stores the x value for pixel drwing for oled
int encData[4]; //array to store encoder value from slaves




#endif