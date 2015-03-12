#include "due_tlc5940.h"
#include "due_tlc5940_config.h"
#include "due_usart.h"
#include "due_timer_counters.h"

/* -----------------------------------------------------------------------
 * Programmer:   Franco Grassano - Mateo Ferley
 * Date:         May 11, 2014
 * Platform:     Arduino DUE
 * Project:      Free DAW
 * References:   Madeline's Makery TLC5940 for DUE library
 *               https://code.google.com/p/arduino-due-tlc5940/
 *               Cody Hazelwood's MIDI controller with motorized faders
 *               http://www.hazelwoodsound.com/motorized-faders-and-the-arduino/ 
 *               Native Capacitive Sensor without additional hardware
 *               http://playground.arduino.cc/Code/CapacitiveSensor
 *               TLC5940 Datasheet
 *               https://www.sparkfun.com/datasheets/Components/General/tlc5940.pdf
 *               L293D Datasheet
 *               http://pdf.datasheetcatalog.com/datasheet/texasinstruments/l293d.pdf
 *               Motorized Fader ALPS - Datasheet
 *               https://cdn-reichelt.de/documents/datenblatt/B400%2FRS60N11M9.pdf
 *               
 * -----------------------------------------------------------------------
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------
 */
/*
    Basic Pin setup for TLC5940:
    ------------                                  ---u----
ARDUINO DUE PINOUT                                            TLC5940                                                       L293D
------------------                                            --------                                                     --------
              SCL1|                    LD293D (pin 2) <-OUT1 |1     28| OUT0 -> LD293D (pin 7)             5V  <- ENABLE1 |1     16| +V -> +5V
              SDA1|-> SCLK  (pin 25)                    OUT2 |2     27| VPRG -> GND               TLC5940 (pin 1) <- IN1  |2     15| IN4
              AREF|                                     OUT3 |3     26| SIN  -> pin 18 - TX1         Motor (pin1) <- OUT1 |3     14| OUT4
               GND|                                     OUT4 |4     25| SCLK -> pin SDA1                             GND  |4     13| GND
                13|                                       .  |5     24| XLAT -> pin 10                               GND  |5     12| GND
                12|                                       .  |6     23| BLANK-> pin 9               Motor (pin1) <-  OUT2 |6     11| OUT3
                11|-> GSCLK (pin 18)                      .  |7     22| GND                       TLC5940 (pin 2) <- IN2  |7     10| IN3        
                10|-> XLAT  (pin 24)                      .  |8     21| VCC  -> +5V                          9V <- Vmotor |8      9| ENABLE2
                 9|-> BLANK (pin 23)                      .  |9     20| -> 10K Resistor -> GND                             --------
                 8|                                       .  |10    19| DCPRG-> +5V
                 7|                                       .  |11    18| GSCLK-> pin 11
                 6|                                       .  |12    17| SOUT
                 5|                                       .  |13    16| XERR
                 .|                                     OUT14|14    15| OUT15  
                 .|                                           --------      
           (RX2)17|                                     
           (TX1)18|-> SIN   (pin 26)                             
           (RX1)19| 
                 .|
                 .|         
------------------                                  
    
    -  The 2K resistor between TLC pin 20 (BLANK) and GND will let ~20mA through each
       LED.  To be precise, it's I = 39.06 / R (in ohms).  This doesn't depend
       on the LED driving voltage.
    - (Optional): put a pull-up resistor (~10k) between +5V and BLANK so that
                  all the LEDs will turn off when the Arduino is reset.

    If you are daisy-chaining more than one TLC, connect the SOUT of the first
    TLC to the SIN of the next.  All the other pins should just be connected
    together:
        BLANK on Arduino -> BLANK of TLC1 -> BLANK of TLC2 -> ...
        XLAT on Arduino  -> XLAT of TLC1  -> XLAT of TLC2  -> ...
    The one exception is that each TLC needs it's own resistor between pin 20
    and GND.

/************************************************************************
*/
//#include <MIDI.h>             //Library for receiving MIDI messages

#define STOP 0
#define BACKWARDS 1
#define FORWARD 2

#define SPEED_STEP 12
#define MIN_SPEED 1280

//Arduino Pin Assignments
const int motorDown    = 0;   //H-Bridge control to make the motor go down
const int motorUp      = 1;   //H-Bridge control to make the motor go up

//Inputs
// DIGITAL
const int touchPin = 2;   //Receive pin for Capacitance Sensing Circuit (Digital 8)
// ANALOG
const int faderPosPin  = 9;   //Position of fader relative to GND (Analog 0)
const int setPin = 10;

//Variables
double   faderMax        = 900;     //Value read by fader's maximum position (0-1023)
double   faderMin        = 3;     //Value read by fader's minimum position (0-1023)
int      faderChannel    = 1;     //Value from 1-8
int      setPoint        = 0;
int      updateValue     = 0;
int      incomingByte    = 0;

int     updateReady     = 0;
bool     touched         = false; //Is the fader currently being touched?
bool     positionUpdated = false; //Since touching, has the MIDI position been updated?

// constants won't change. Used here to set a pin number :
const int ledPin =  13;      // the number of the LED pin

void setup() {
  //	MIDI.begin(MIDI_CHANNEL_OMNI);  //Receive messages on all MIDI channels
  //	MIDI.turnThruOff();             //We don't need MIDI through for this
  pinMode(ledPin, OUTPUT);
  
  Serial.begin(9600);
    // INICIALIZO LA LIBRERIA PARA EL MANEJO DE TLC5940
  initTLC5940();
  
  // Establis faderMin & faderMax
  calibrateFader();
  

  //	attachInterrupt(0, nextChannel, RISING);
  //	attachInterrupt(1, prevChannel, RISING);
}

void loop() {
  /* If there is a MIDI message waiting, and it is for the currently selected
   	   fader, and it is a PitchBend message (used for fader control), then convert
   	   the PitchBend value and update the fader's current position.  */
  //if (MIDI.read() && MIDI.getChannel() == faderChannel && MIDI.getType() == PitchBend ) {
  /* Bitwise math to take two 7 bit values for the PitchBend and convert to
   		   a single 14 bit value.  Then converts it to value between 0 and 1023
   		   to control the fader. */
  //	int value = (((MIDI.getData2() << 7) + MIDI.getData1()) * 0.0625);
  
   // send data only when you receive data:
   if (Serial.available() > 0) {
     // read the incoming byte:
     incomingByte = Serial.read();
     
//     if (!updateReady){
//       updateValue = 0 | (incomingByte << 7);
//       updateReady = 1;
//     }
//     else if (updateReady == 1){
//       updateValue |= incomingByte;
//       updateValue = map(updateValue,0,16383,0,1023);
//       updateReady = 2;
//    }
     updateValue = map(incomingByte,0,255,0,1023);
    
   }
    
//  if (updateReady == 2){ 
     updateFader(updateValue);
//     //Serial.println(updateValue);
//     updateReady = 0;
//  }
  
  checkTouch();  //Checks to see if the fader is being touched
  
  //If the fader has been touched, it needs to update the position on the MIDI host
  /*	if (!positionUpdated) {
   		updateFaderMidi();
   		positionUpdated = true;
   	}
   */
}

//Function to move fader to a specific position between 0-1023 if it's not already there
void updateFader(int posicion) {
  static int fadSpeed = 10;
  static int estado = 0;

  int posFader = analogRead(faderPosPin);
//  Serial.print("Lectura ADC: ");
//  Serial.println(posFader);     
  //Serial.println(analogRead(faderPosPin));
  if (touched){
    estado = STOP;
    fadSpeed = 0;
    setGSData (motorDown, fadSpeed);
    setGSData (motorUp, fadSpeed);
    sendGSData();
  }
  switch (estado) {
    case STOP:
      if (posicion < posFader - 20 && posFader > faderMin && !touched) {
        estado = BACKWARDS;
        fadSpeed = MIN_SPEED;
        setGSData (motorDown, fadSpeed);
        sendGSData();
        break;
      }
      else if (posicion > posFader + 20 && posFader < faderMax && !touched) {
        estado = FORWARD;
        fadSpeed = MIN_SPEED;
        setGSData (motorUp, fadSpeed);
        sendGSData();
        break;
      }
    case BACKWARDS:
      if (posicion > posFader - 5) {  
        estado = STOP;
        fadSpeed = 0;
        setGSData (motorDown, fadSpeed);
        setGSData (motorUp, fadSpeed);
        sendGSData();
        break;
      }
      if (fadSpeed < 4095)
        fadSpeed = fadSpeed + SPEED_STEP;
      else
        fadSpeed = 4095;
      setGSData (motorDown, fadSpeed);
      sendGSData();
      break;
    case FORWARD:
      if (posicion < posFader + 5) {
        estado = STOP;
        fadSpeed = 0;
        setGSData (motorUp, fadSpeed);
        setGSData (motorDown, fadSpeed);
        sendGSData();
        break;
      }
      if (fadSpeed < 4095)
        fadSpeed = fadSpeed + SPEED_STEP;
      else
        fadSpeed = 4095;
      setGSData (motorUp, fadSpeed);
      sendGSData();
      break;
  }
  
  /*
  if (posicion < analogRead(faderPosPin) - 10 && posicion > faderMin && !touched) {
    analogWrite(motorDown, fadSpeed);
    while (posicion < analogRead(faderPosPin) - 20 && !touched) {
    };  //Loops until motor is done moving
    analogWrite(motorDown, LOW);
  }
  else if (posicion > analogRead(faderPosPin) + 10 && posicion < faderMax && !touched) {
    analogWrite(motorUp, fadSpeed);
    while (posicion > analogRead(faderPosPin) + 20 && !touched) {
    }; //Loops until motor is done moving
    analogWrite(motorUp, LOW);
  }
  */
}

void updateFaderMidi() {
  int  velocity    = faderPosition();
  byte channelData = 0xE0 + (faderChannel - 1);
  // MIDI Message:
  Serial.write(channelData);               //  E(PitchBend)  Channel (0-9)
  Serial.write(velocity & 0x7F);           //  Least Sig Bits of Data
  Serial.write((velocity >> 7) & 0x7F);    //  Most  Sig Bits of Data
}

//Calibrates the min and max position of the fader
void calibrateFader() {
  //Send fader to the top and read max position
  setGSData (motorUp, 4095);
  sendGSData();
  delay(250);
  setGSData (motorUp, 0);
  sendGSData();
  faderMax = analogRead(faderPosPin);

  //Send fader to the bottom and read min position
  setGSData (motorDown, 4095);
  sendGSData();
  delay(250);
  setGSData (motorDown, 0);
  sendGSData();
  faderMin = analogRead(faderPosPin);
}

//Returns a MIDI pitch bend value for the fader's current position
//Cases ensure that there is a -infinity (min) and max value despite possible math error
int faderPosition() {
  int position = analogRead(faderPosPin);
  int returnValue = 0;

  if (position <= faderMin) {
    returnValue = 0;
  }
  else if (position >= faderMax) {
    returnValue = 16383;
  }
  else {
    returnValue = ((float)(position - faderMin) / (faderMax - faderMin)) * 16383;
  }

  return returnValue;
}

//Check to see if the fader is being touched
void checkTouch() {
  //For the capSense comparison below,
  //700 is arbitrary and may need to be changed
  //depending on the fader cap used (if any).

  if (!touched && readCapacitivePin(touchPin) >= 5) {
    touched = true;

    //Send MIDI Touch On Message
    //		Serial.write(0x90);
    //		Serial.write(0x67 + faderChannel);
    //		Serial.write(0x7f);
  }
  else if (touched && readCapacitivePin(touchPin) < 5) {
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

// readCapacitivePin
//  Input: Arduino pin number
//  Output: A number, from 0 to 17 expressing
//  how much capacitance is on the pin
//  When you touch the pin, or whatever you have
//  attached to it, the number will get higher
uint8_t readCapacitivePin(int pinToMeasure) {
  pinMode(pinToMeasure, OUTPUT);
  digitalWrite(pinToMeasure, LOW);
  delay(1);
  // Prevent the timer IRQ from disturbing our measurement
  noInterrupts();
  // Make the pin an input with the internal pull-up on
  pinMode(pinToMeasure, INPUT_PULLUP);

  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.
  uint8_t cycles = 17;
       if (digitalRead(pinToMeasure)) { cycles =  0;}
  else if (digitalRead(pinToMeasure)) { cycles =  1;}
  else if (digitalRead(pinToMeasure)) { cycles =  2;}
  else if (digitalRead(pinToMeasure)) { cycles =  3;}
  else if (digitalRead(pinToMeasure)) { cycles =  4;}
  else if (digitalRead(pinToMeasure)) { cycles =  5;}
  else if (digitalRead(pinToMeasure)) { cycles =  6;}
  else if (digitalRead(pinToMeasure)) { cycles =  7;}
  else if (digitalRead(pinToMeasure)) { cycles =  8;}
  else if (digitalRead(pinToMeasure)) { cycles =  9;}
  else if (digitalRead(pinToMeasure)) { cycles = 10;}
  else if (digitalRead(pinToMeasure)) { cycles = 11;}
  else if (digitalRead(pinToMeasure)) { cycles = 12;}
  else if (digitalRead(pinToMeasure)) { cycles = 13;}
  else if (digitalRead(pinToMeasure)) { cycles = 14;}
  else if (digitalRead(pinToMeasure)) { cycles = 15;}
  else if (digitalRead(pinToMeasure)) { cycles = 16;}

  // End of timing-critical section
  interrupts();

  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to 
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  digitalWrite(pinToMeasure, LOW);
  pinMode(pinToMeasure, OUTPUT);

  return cycles;
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

