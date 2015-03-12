
//Function to move fader to a specific position between 0-1023 if it's not already there
void updateFader(int destino) {	
  static int fadSpeed = MIN_SPEED;    // PWM value
  static int faderState = STOP;       // state
  int posFader = analogRead(faderPosPin);

  //destino = map(destino, faderMin, faderMax, 0, 1024);

  switch(faderState){
  case STOP:
    if (destino < posFader - STOP_TH && posFader > faderMin && !touched){ 
      faderState = BACKWARDS;
      fadSpeed = MIN_SPEED;
      Tlc.set(DOWN,fadSpeed);
      Tlc.update();
      break;    
    }  
    else if (destino > posFader + STOP_TH && posFader < faderMax && !touched) {
      faderState = FORWARD;
      fadSpeed = MIN_SPEED;       
      Tlc.set(UP,fadSpeed);
      Tlc.update();   
      break;   
    }  
    else if (touched){
      faderState = TOUCHED;
      fadSpeed = OFF;
      Tlc.set(UP,fadSpeed);
      Tlc.set(DOWN,fadSpeed);
      Tlc.update();  
      break;    
    }
  case BACKWARDS:
    if(destino > posFader - STOP_TH && !touched){
      faderState = STOP;
      fadSpeed = OFF;       
      Tlc.set(DOWN,fadSpeed);
      Tlc.update();  
      break;    
    }
    if(fadSpeed < MAX_SPEED){     
      if (abs(destino-posFader)>10){      
        //fadSpeed = MIN_SPEED + (MAX_SPEED-MIN_SPEED)*(1-exp(-abs(posFader)/TAU));
        fadSpeed = MIN_SPEED + (MAX_SPEED-MIN_SPEED)*(1-exp(-abs(destino-posFader)/TAU));        
        //fadSpeed = fadSpeed + SPEED_STEP;
      }
      else{
        fadSpeed = MIN_SPEED;      // max speed is 0, so approaching to 0 increases speed
      }
    }
    else{
      fadSpeed = MAX_SPEED;
    }
    Tlc.set(DOWN,fadSpeed);
    Tlc.update();  
    break;
  case FORWARD:
    if(destino < posFader + STOP_TH && !touched){
      faderState = STOP;
      fadSpeed = OFF;        
      Tlc.set(UP,fadSpeed);
      Tlc.update();      
      break;
    }
    if(fadSpeed < MAX_SPEED){     
      if (abs(destino-posFader)>10){      
        //fadSpeed = MIN_SPEED + (MAX_SPEED-MIN_SPEED)*(1-exp(-abs(posFader)/TAU));
        fadSpeed = MIN_SPEED + (MAX_SPEED-MIN_SPEED)*(1-exp(-abs(destino-posFader)/TAU));
        //fadSpeed = fadSpeed + SPEED_STEP;
      }
      else{
        fadSpeed = MIN_SPEED;      // max speed is 0, so approaching to 0 increases speed
      }
    }
    else{
      fadSpeed = MAX_SPEED;
    }

    Tlc.set(UP,fadSpeed);
    Tlc.update(); 
    break;
  case TOUCHED:
    if(!touched)
      faderState = STOP;
  }

}



//Calibrates the min and max position of the fader
void calibrateFader() {
  //Send fader to the top and read max position
  //digitalWrite(motorUp, HIGH);
  // Tlc.clear();
  Tlc.set(UP,MAX_SPEED);
  Tlc.update();
  delay(150);

  //digitalWrite(motorUp, LOW);	
  Tlc.set(UP,OFF);
  Tlc.update();
  faderMax = analogRead(faderPosPin) - 5;

  //Send fader to the bottom and read min position
  //digitalWrite(motorDown, HIGH);
  Tlc.set(DOWN,MAX_SPEED);
  Tlc.update();
  delay(150);
  //digitalWrite(motorUp, LOW);	
  Tlc.set(DOWN,OFF);
  Tlc.update();
  //digitalWrite(motorDown, LOW);
  faderMin = analogRead(faderPosPin) + 5;
}



//Check to see if the fader is being touched
void checkTouch() {
  //For the capSense comparison below,
  //700 is arbitrary and may need to be changed
  //depending on the fader cap used (if any).

  if (!touched && touchLine.capacitiveSensor(20) >= 700) {
    touched = true;

    //Send MIDI Touch On Message
    //		Serial.write(0x90);
    //		Serial.write(0x67 + faderChannel);
    //		Serial.write(0x7f);
  }
  else if (touched && touchLine.capacitiveSensor(20) < 700) {
    touched = false;

    //Send MIDI Touch Off Message
    //		Serial.write(0x90);
    //		Serial.write(0x67 + faderChannel);
    //		Serial.write((byte) 0x00);
  }

  if (touched) {
    positionUpdated = false;
  }
}


//Selects the next channel in the DAW
void nextChannel() {
  static unsigned long last_interrupt0_time = 0;      //Interrupt Debouncing
  unsigned long interrupt0_time = millis();           //Interrupt Debouncing

  if (interrupt0_time - last_interrupt0_time > 200) { //Interrupt Debouncing
    if (faderChannel < 8) {
      faderChannel++;

      Serial.write(0x90);
      Serial.write(0x17 + faderChannel);
      Serial.write(0x7f);                         //Note On
      Serial.write(0x90);
      Serial.write(0x17 + faderChannel);
      Serial.write((byte) 0x00);		            //Note Off
    }
  }

  last_interrupt0_time = interrupt0_time;             //Interrupt Debouncing
}

//Selects the previous channel in the DAW
void prevChannel() {
  static unsigned long last_interrupt1_time = 0;      //Interrupt Debouncing
  unsigned long interrupt1_time = millis();           //Interrupt Debouncing

  if (interrupt1_time - last_interrupt1_time > 200) { //Interrupt Debouncing
    if (faderChannel > 1) {
      faderChannel--;

      Serial.write(0x90);
      Serial.write(0x17 + faderChannel);
      Serial.write(0x7f);                         //Note On
      Serial.write(0x90);
      Serial.write(0x17 + faderChannel);
      Serial.write((byte) 0x00);		            //Note Off
    }
  }

  last_interrupt1_time = interrupt1_time;             //Interrupt Debouncing
}

