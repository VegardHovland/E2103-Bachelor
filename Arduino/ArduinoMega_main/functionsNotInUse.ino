// test a given trajectory, loop through an array of angles
void followTrajTest() {
  float traj[20] = {0, 20, 90, -180, 90, 50, 100, 20, -40, 50, 200, 110, 30, -120, -180, 0, -20, 10, -50, 80};
  for (int i = 0 ; i < 20; i++) {                              //Get offset speed for calibration calculations, write to monitor
    int shutdowntime = millis() + 1000;                        //Gives actuator time to reach setpoint
    int currTime = millis();
    actuators[2].setSetpoint(traj[i]);
    while (shutdowntime > currTime) {                         // Comutes pid for actuators during shutdown time
      controllActuators(actuators);                           // Pid controll on all the acuators
      currTime = millis();
      Serial.println(actuators[2].getAngle());
    }
  }
}

// Perform a step respons
void stepResponse() {
  delay(100);
  actuators[3].readAngle();
  while ( actuators[3].getAngle() < 300) {
    md2.setM2Speed(400);                                        // Motor 4 is driver 2 M2
    actuators[3].readAngle();
    Serial.println(actuators[3].getAngle());
  }
  md2.setM2Speed(0);                                            // stop Motor 4, is driver 2 M2
}

//Find offset from pid that is lost through gear ratio 150:1
void calibrationData() {
  for (int i = 0; i < numActuators; i++) {
    actuators[i].setSetpoint(startPos[i]);                      // Updates setpoints to startposition
  }
  int shutdowntime = millis() + 5000;                           //Generate a shutdown time
  int currTime = millis();
  while (shutdowntime > currTime) {                             // Comutes pid for actuators during shutdown time
    controllActuators(actuators);                               // Pid controll on all the acuators
    currTime = millis();
  }

  for (int i = 0 ; i < numActuators; i++) {                     //Get offset speed for calibration calculations, write to monitor
    Serial.print(i);
    Serial.println(" :");
    for (int j = 0; j <= 100; j++) {
      int shutdowntime = millis() + 100;                        //Gives actuator time to reach setpoint
      int currTime = millis();
      actuators[i].setSetpoint(j);
      while (shutdowntime > currTime) {                         // Comutes pid for actuators during shutdown time
        controllActuators(actuators);                           // Pid controll on all the acuators
        currTime = millis();
      }
      Serial.println(actuators[i].getEffort());
    }
  }
}
void serialPrintData() {
  for (int i = 0; i < numActuators; i++) {
    Serial.print(i + 1);                                         //Print motor nr (1-4)
    Serial.println(" : ");

    Serial.print("angle ");                                      // Prints joint angle in serial monitor
    Serial.print(": ");
    Serial.println(actuators[i].getAngle());

    Serial.print("setpoint ");
    Serial.print(i + 1);                                         // Prints setpoint in serial monitor
    Serial.print(": ");
    Serial.println(actuators[i].getSetpoint());

    Serial.print("Kp: ");                                        //Print parameters
    Serial.println(actuators[i].getKp());
    Serial.print("Ti: ");
    Serial.println(actuators[i].getTi());

  }
  int amps [4];
  amps[0] = md1.getM1CurrentMilliamps();                         // Get current for motor 1
  amps[1] = md1.getM2CurrentMilliamps();                         // Get current for motor 2
  amps[2] = md2.getM1CurrentMilliamps();                         // Get current for motor 3
  amps[3] = md2.getM2CurrentMilliamps();                         // Get current for motor 4

  for (int i = 0; i < numActuators; i++) {
    Serial.print("current ");
    Serial.print(i + 1);                                         // Prints the motors current i serial
    Serial.print(": ");
    Serial.println(amps[i]);
  }
}

void setupOled() {
  //OLED SETUP for display 1
  if (!display_1.begin(SSD1306_SWITCHCAPVCC, oled1)) {            // Start display and chack if sucess
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display_1.setTextSize(2);                                       // Set display properties and clear display
  display_1.setTextColor(WHITE);
  display_1.setCursor(0, 0);
  display_1.clearDisplay();

  //OLED SETUP for display 2
  if (!display_2.begin(SSD1306_SWITCHCAPVCC, oled2)) {            // Start display and chack if sucess
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display_2.setTextSize(2);;                                      // Set display properties and clear display
  display_2.setTextColor(WHITE);
  display_2.setCursor(0, 0);
  display_2.clearDisplay();
}
//Draw graph on oled 2
void drawGraph(Actuator actuators[]) {
  if (pixelX > 128) {                                             // If at end of display
    pixelX = 0;                                                   // Reset x position
    display_2.clearDisplay();                                     // Clear display
  }

  for (int i = 0; i < numActuators; i++) {                        // Loop over mact 1-4
    int y = map(actuators[i].getAngle(), 0, 360, 0 , 64);         // Get y coordinate
    display_2.drawPixel(pixelX , y, WHITE);                       // Print a pixel at x,y
    display_2.display();                                          // Display on display
  }
  pixelX++;                                                       // Increment x position
}


//Print text on oled 1
void printText(Actuator actuators[]) {
  int currtime = millis();                                          // Store current time
  if ( (currtime - printPrevtime) > 500) {                          // Refresh display every .5 sek
    display_1.setTextSize(2);                                       // Chose text size
    display_1.setTextColor(WHITE);                                  // Choose couler
    display_1.setCursor(0, 0);                                      // Return to upper left corner
    display_1.clearDisplay();                                       // Clear old data
    for (int i = 0; i < numActuators; i++) {                        // Loop over act 1-4
      display_1.print(i + 1);                                       // Prints information to display
      display_1.print(": ");
      display_1.println(actuators[i].getAngle());
      display_1.display();                                          // Display on display
    }
    printPrevtime = millis();                                       // Store previous refresh time
  }
}

//This prints data we want to plot in serial plot, in serial monitor choose serial plotter
void serialPlot() {                                          //We will tune the parameters using the first motor
  //int currTime = millis();                                 // Calculate scan time
  //int scantime = currTime - prevtime;

  //Serial.println(scantime);
  //Serial.println(actuators[2].getSetpoint());           // print motor 1 setpoint
  //Serial.println(actuators[2].getAngle());              // Print motor 1 angle
  Serial.println(actuators[0].getEffort());                 // Print motor 1 speed
  //prevtime = currTime;                                    // Update prev time
}
