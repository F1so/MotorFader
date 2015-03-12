
/* -----------------------------------------------------------------------
 * Programmer:   Cody Hazelwood
 * Date:         May 17, 2012
 * Platform:     Arduino Uno
 * Description:  MIDI controller using a motorized fader and an Arduino
 * Dependencies: CapSense Arduino Library (for fader touch sensitivity)
 *               http://www.arduino.cc/playground/Main/CapSense
 *               MIDI Library for Arduino
 *               http://www.arduino.cc/playground/Main/MIDILibrary
 * -----------------------------------------------------------------------
 * Copyright Ã‚Â© 2012.  Cody Hazelwood.
 *              
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
    Basic Pin setup:
 ------------                                  ---u----
 ARDUINO   13|-> SCLK (pin 25)           OUT1 |1     28| OUT channel 0
 12|                           OUT2 |2     27|-> GND (VPRG)
 11|-> SIN (pin 26)            OUT3 |3     26|-> SIN (pin 11)
 10|-> BLANK (pin 23)          OUT4 |4     25|-> SCLK (pin 13)
 9|-> XLAT (pin 24)             .  |5     24|-> XLAT (pin 9)
 8|                             .  |6     23|-> BLANK (pin 10)
 7|                             .  |7     22|-> GND
 6|                             .  |8     21|-> VCC (+5V)
 5|                             .  |9     20|-> 2K Resistor -> GND
 4|                             .  |10    19|-> +5V (DCPRG)
 3|-> GSCLK (pin 18)            .  |11    18|-> GSCLK (pin 3)
 2|                             .  |12    17|-> SOUT
 1|                             .  |13    16|-> XERR
 0|                           OUT14|14    15| OUT channel 15
 ------------                                  --------
 
 -  Put the longer leg (anode) of the LEDs in the +5V and the shorter leg
 (cathode) in OUT(0-15).
 -  +5V from Arduino -> TLC pin 21 and 19     (VCC and DCPRG)
 -  GND from Arduino -> TLC pin 22 and 27     (GND and VPRG)
 -  digital 3        -> TLC pin 18            (GSCLK)
 -  digital 9        -> TLC pin 24            (XLAT)
 -  digital 10       -> TLC pin 23            (BLANK)
 -  digital 11       -> TLC pin 26            (SIN)
 -  digital 13       -> TLC pin 25            (SCLK)
 -  The 2K resistor between TLC pin 20 and GND will let ~20mA through each
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
 
 This library uses the PWM output ability of digital pins 3, 9, 10, and 11.
 Do not use analogWrite(...) on these pins.
 
 This sketch does the Knight Rider strobe across a line of LEDs.
 
 Alex Leone <acleone ~AT~ gmail.com>, 2009-02-03 */

/************************************************************************
 *   Currently known issues:
 *
 *   *  Only DAW Channels 1-8 work right now
 *   *  Better Fader Calibration is needed
 *   *  Doesn't work with Pro Tools due to Avid's restrictions (but any
 *         other DAW that supports Mackie Control without HUI support will
 *         work fine (Cubase, Logic, Digital Performer, Live, etc.)
 *
 *************************************************************************/

#include <CapacitiveSensor.h>         //Library for fader touch sensitivity
#include <MIDI.h>             //Library for receiving MIDI messages
#include "Tlc5940.h"

#define STOP          0
#define BACKWARDS     1
#define FORWARD       2
#define TOUCHED       3

// TLC defines
#define OFF           0    
#define UP            0      // Channel 0 (TLC) - Motor Up -> Pin 2 (L293D) /// OUT Pin 3 -> Pin 1 Motor
#define DOWN          1      // Channel 1 (TLC) - Motor Down -> Pin7 (L293D) /// OUT Pin 6 -> Pin 2 Motor

#define STOP_TH       10
#define SPEED_STEP    2

#define MIN_SPEED     64
#define MAX_SPEED     128
#define TAU           128

//Arduino Pin Assignments
const int motorDown    = 9;   //H-Bridge control to make the motor go down
const int motorUp      = 10;   //H-Bridge control to make the motor go up

//Inputs

// DIGITAL
const int touchSend    = 4;   //Send pin for Capacitance Sensing Circuit (Digital 7)
const int touchReceive = 2;   //Receive pin for Capacitance Sensing Circuit (Digital 8)
const int VPRG_pin     = 6;

// ANALOG
const int faderPosPin  = 3;   //Position of fader relative to GND (Analog 0)
const int setPin = 0;

//Variables
unsigned int dutyStart = 0;
unsigned int dutyEnd = 0;
unsigned int startMillis = 0;
unsigned int endMillis = 0;

int      value           = 0;
int      aux             = 0;
double   faderMax        = 900;     //Value read by fader's maximum position (0-1023)
double   faderMin        = 3;     //Value read by fader's minimum position (0-1023)
int      faderChannel    = 1;     //Value from 1-8
int      setPoint        = 0;
int      i               = 0;
bool     touched         = false; //Is the fader currently being touched?
bool     positionUpdated = false; //Since touching, has the MIDI position been updated?

CapacitiveSensor touchLine     = CapacitiveSensor(touchSend, touchReceive);

void setup() {
  //MIDI.begin(MIDI_CHANNEL_OMNI);  //Receive messages on all MIDI channels
  //MIDI.turnThruOff();             //We don't need MIDI through for this

  // Initialize TLC5940 library
  Tlc.init();

  // Establish serial baud rate at 9600 bauds
  Serial.begin(9600);

  // Set pin 9's PWM frequency to 31250 Hz (31250/1 = 31250)
  //  setPwmFrequency(motorDown, 1);
  // Set pin 10's PWM frequency to 31250 Hz (31250/1 = 31250)
  //  setPwmFrequency(motorUp, 1);

  // Establish faderMin & faderMax
  calibrateFader();

  //	attachInterrupt(0, nextChannel, RISING);
  //	attachInterrupt(1, prevChannel, RISING);

} 

void loop() {
  //Tlc.clear();

  /* If there is a MIDI message waiting, and it is for the currently selected
   	   fader, and it is a PitchBend message (used for fader control), then convert
   	   the PitchBend value and update the fader's current position.  */
  //  if (MIDI.read() && MIDI.getChannel() == faderChannel && MIDI.getType() == PitchBend ) {
  //  /* Bitwise math to take two 7 bit values for the PitchBend and convert to
  //   		   a single 14 bit value.  Then converts it to value between 0 and 1023
  //   		   to control the fader. */
  //  	int value = (((MIDI.getData2() << 7) + MIDI.getData1()) * 0.0625);
  //Serial.println(analogRead(setPin));
  
  updateFader(analogRead(setPin));    // Read value from pot
  Serial.println(analogRead(setPin));
  
  //
  //  }
  //  Serial.println("UP - CHANNEL 0");
  //  Tlc.set(UP,128); Tlc.update(); delay(1000);
  //  Tlc.set(UP,0); Tlc.update(); 
  //  Serial.println("DOWN - CHANNEL 1");
  //  Tlc.set(DOWN,128); Tlc.update(); delay(1000);
  //  Tlc.set(DOWN,0); Tlc.update(); 
  //  Serial.println("RED LED - CHANNEL 2");
  //  Tlc.set(2,4095); Tlc.update(); delay(1000);
  //  Tlc.set(2,0); Tlc.update(); 

  checkTouch();  //Checks to see if the fader is being touched

  //If the fader has been touched, it needs to update the position on the MIDI host
  if (!positionUpdated) {	
    //  	updateFaderMidi();
    //        Serial.println(1,DEC);
    Serial.println(analogRead(faderPosPin)>>2);
    //        Serial.println();
    delay(40);
    positionUpdated = true;
  }     	
}


void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
    case 1: 
      mode = 0x01; 
      break;
    case 8: 
      mode = 0x02; 
      break;
    case 64: 
      mode = 0x03; 
      break;
    case 256: 
      mode = 0x04; 
      break;
    case 1024: 
      mode = 0x05; 
      break;
    default: 
      return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } 
    else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } 
  else if(pin == 3 || pin == 11) {
    switch(divisor) {
    case 1: 
      mode = 0x01; 
      break;
    case 8: 
      mode = 0x02; 
      break;
    case 32: 
      mode = 0x03; 
      break;
    case 64: 
      mode = 0x04; 
      break;
    case 128: 
      mode = 0x05; 
      break;
    case 256: 
      mode = 0x06; 
      break;
    case 1024: 
      mode = 0x7; 
      break;
    default: 
      return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

