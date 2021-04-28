//Defines
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define oled1 0x3C
#define oled2 0x3D

//Variables------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
const int numActuators = 4 ;        // Defines the number of actuators
int printPrevtime;
int prevtime = 0;
int pixelX = 0;                     // Stores the x value for pixel drwing for oled
float startPos[4] = {0 , 0, -90, 0};
int shutdowntime;

const char robot_id = "robotleg";                                                                                   // Robot namespace for topic
const char* joint_name[4] = {"joint1", "joint2", "joint3", "joint4"};       // Name of joints for topic


//Driver 1, for actuator 1 and 2
unsigned char M11nSLEEP = 8;        // Common sleep pin for all 4 actuators
unsigned char M11DIR = 3;           // Directio pin for actuator 1, High=current from A to B, low B to A
unsigned char M11PWM = 4;           // PWM pin for actuator 1, pwm fq=980
unsigned char M11nFAULT = 9;        // Common fault pin fot all 4 acutators
unsigned char M11CS = A0;           // Pin for current sensing for actuator 1

unsigned char M12nSLEEP = 8;        // Common sleep pin for all 4 actuators
unsigned char M12DIR = 5;           // Directio pin for actuator 2, High=current from A to B, low B to A
unsigned char M12PWM = 13;          // PWM pin for actuator 2, pwm fq=980
unsigned char M12nFAULT = 9;        // Common fault pin fot all 4 acutators
unsigned char M12CS = A1;           // Pin for current sensing for actuator 2

//Driver 2, for actuator 3 and 4
unsigned char M21nSLEEP = 8;        // Common sleep pin for all 4 actuators
unsigned char M21DIR = 6;           // Directio pin for actuator 3, High=current from A to B, low B to A
unsigned char M21PWM = 10;          // PWM pin for actuator 3, pwm fq=490
unsigned char M21nFAULT = 9;        // Common fault pin fot all 4 acutators
unsigned char M21CS = A2;           // Pin for current sensing for actuator 3

unsigned char M22nSLEEP = 8;        // Common sleep pin for all 4 actuators
unsigned char M22DIR = 7;           // Directio pin for actuator 4, High=current from A to B, low B to A
unsigned char M22PWM = 11;          // PWM pin for actuator 4, pwm fq=490
unsigned char M22nFAULT = 9;        // Common fault pin fot all 4 acutators
unsigned char M22CS = A3;           // Pin for current sensing for actuator 4
