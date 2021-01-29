//This will be a simple program to test that actuator controll is working before we test the larger programm


void setup()
{
	Serial.begin(9600);
}

void loop()
{

//Get angle //Read from encoder // calc 

	 currentTime = millis();                //get current time
     elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
    error = _setPoint - angle;                                // determine error
    cumError += error * elapsedTime;                // compute integral
    rateError = (error - lastError)/elapsedTime;   // compute derivative
 
    double out = _kp*error + _ki*cumError + _kd*rateError;                //PID output               
        
    if (out > maxAngle)
     {
         out = maxAngle;
     }

    if (out < minAngle)
     {
         out = minAngle;
     }
        
        
    lastError = error;                                //remember current error
    previousTime = currentTime;                        //remember current time
 
                                          //have function return the PID output

    //ANalog write desired angle // controll speed
}
