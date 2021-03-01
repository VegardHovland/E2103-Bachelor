#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <stdlib.h>
//Constants:
const int potPin1 = A1; //pin A0 to read analog input
const int potPin2 = A2; //pin A0 to read analog input
const int potPin3 = A3; //pin A0 to read analog input
const int potPin4 = A4; //pin A0 to read analog input
//Variables:
float value1; //save analog value
float value2; //save analog value1
float value3; //save analog value
float value4; //save analog value

ros::NodeHandle nh;
sensor_msgs::JointState robot_state;
ros::Publisher pub("joint_states", &robot_state);

float angle[4] = {20.4,10.2,5.5,20.0};
float ut;

void setup()
{ 
  pinMode(potPin1, INPUT); //Optional 
  pinMode(potPin2, INPUT); //Optional 
  pinMode(potPin3, INPUT); //Optional 
  pinMode(potPin4, INPUT); //Optional 
  
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub);
}

void loop()
{
  nh.spinOnce();
  value1 =  analogRead(potPin1);          //Read and save analog value from potentiometer
  value1 =  map(value1, 0, 1023, 0, 3.14); //Map value 0-1023 to 0-255 (PWM)

  value2 =  analogRead(potPin2);           //Read and save analog value from potentiometer
  value2 =  map(value2, 0, 1023, 0, 3.14); //Map value 0-1023 to 0-255 (PWM)

  value3 =  analogRead(potPin3);           //Read and save analog value from potentiometer
  value3 =  map(value3, 0, 1023, 0, 3.14); //Map value 0-1023 to 0-255 (PWM)

  value4 =  analogRead(potPin4);           //Read and save analog value from potentiometer
  value4 =  map(value4, 0, 1023, 0, 3.14); //Map value 0-1023 to 0-255 (PWM)
  
  angle[0] = value1; 
  angle[1] = value2; 
  angle[2] = value3; 
  angle[3] = value4; 
 
  robot_state.position = angle;
  pub.publish( &robot_state);
  nh.spinOnce();
  delay(100);
}
