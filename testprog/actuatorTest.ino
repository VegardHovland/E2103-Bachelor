//This will be a simple program to test that actuator controll is working before we test the larger programm 

#include <Arduino.h>

double kp, ki, kd;
double currentTime, elapsedTime, previousTime;
double input, out;
float lastError, error, cumError, rateError;
float setPoint, angle;
int maxAngle = 170;
int minAngle = 0;
int encoderPin = 5;
int actuatorPin = 6;

void setup()
{
	Serial.begin(9600);
    pinMode (encoderPin,INPUT);
    pinMode (actuatorPin, OUTPUT);
}

void loop()
{

    input = analogRead(encoderPin) ;                                     //Reads a value between 0 and 1023
    angle = (input * 360) / 1023 ;                                      //calculates current angle
 
	currentTime = millis();                                             //get current time
    elapsedTime = (double)(currentTime - previousTime);                 //compute time elapsed from previous computation
        
    error = setPoint - angle;                                            // determine error
    cumError += error * elapsedTime;                                     // compute integral
    rateError = (error - lastError)/elapsedTime;                         // compute derivative
 
    out = kp*error + ki*cumError + kd*rateError;                         //PID output               
        
    if (out > maxAngle)                                                  // TROR IKKE DETTE KAN GJØRES SLIK, KOMMER AN PÅ MOTOR VERDIER
     {
         out = maxAngle;
     }

    if (out < minAngle)
     {
         out = minAngle;
     }
        
        
    lastError = error;                           //remember current error
    previousTime = currentTime;                  //remember current time

    analogWrite(actuatorPin, out);               //ANalog write desired angle // controll speed
 
                                        

   
}
