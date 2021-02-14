//Slave program dedicated to a slave arduino for encoder reading
 
 int pinA = 6;                                //Encoder pin for A puls
 int pinB = 7;                                //Encoder pin for B puls
 int outPin = 8;                              //Output pin to Master    

 int counter = 0; 
 int aState;
 int aLastState;  
 double anngle;

 void setup() { 
   pinMode (pinA,INPUT);                     //Defines the input pins
   pinMode (pinB,INPUT);
   
   Serial.begin (9600);                       //Begins the serial monitor
   aLastState = digitalRead(pinA);            // Reads the initial state of the outputA
 } 

 void loop() { 
   aState = digitalRead(pinA);                // Reads the "current" state of the outputA
   if (aState != aLastState){                 // If the previous and the current state of the outputA are different, that means a Pulse has occured
     if (digitalRead(pinB) != aState) {       // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
       counter ++;
     } else {
       counter --;
     }
     Serial.print("Position: ");              //Prints the postion to serial monitor 
     Serial.println(counter);
   } 
   aLastState = aState;                      // Updates the previous state of the outputA with the current state
   //float angle = counter;                     //kommer an på hvilken enkoder vi går for, dette må gjøres etter vi har fått enkoder
   //analogWrite(outPin, angle);
 }
