/*
================================================================================
File name: due_tlc5940_config.h
   System: Due TLC5940 Library
 Platform: Arduino Due (Amtel SAM3X8E ARM Cortex-M3)
   Author: Madeline Usher
  Created: July 5, 2013
  Purpose: Configuration parameters for the Due TLC5940 Library.
================================================================================
  $LastChangedDate: 2013-07-14 11:51:11 -0500 (Sun, 14 Jul 2013) $
  $LastChangedBy: maddy314@gmail.com $
================================================================================
  Copyright (c) 2013 Madeline Usher <maddy314 ~@~ gmail.com>
 
  The Arduino Due TLC5940 Library is free software: you can redistribute it
  and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License, 
  or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT 
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along with 
  this library.  If not, see <http://www.gnu.org/licenses/>.
================================================================================
*/


////////////////////////////////////////////////////////////////////////////////
/// To use this library, copy all the files in this folder into your sketch's
/// folder and edit this file.
///
/// The files have to be in your sketch's directory to allow individual 
/// customization via the #defines in the config file, which is unfortunately
/// necessary -- the name of the interrupt handler function must be known at
/// compile-time, which basically means we're dependent upon #defines to 
/// configure the library.
///
/// You should edit the configuration parameters in this file.
///
/// Because some of the TIOA outputs of the microcontroller's timers are not
/// connected to any Due pins, and because the GSCLK and BLANK timers must be
/// channels within the same timer counter, only some combinations are valid.
/// The following are the only valid configurations:
///
/// GSCLK output on Due pin 2 (digital 2):
///   GSCLK_TC = TC0_CH0
///   BLANK_TC = TC0_CH1
///
/// GSCLK output on Due pin 2 (digital 2):
///   GSCLK_TC = TC0_CH0
///   BLANK_TC = TC0_CH2
///
/// GSCLK output on Due pin 61 (analog 7):
///   GSCLK_TC = TC0_CH1
///   BLANK_TC = TC0_CH0  // can't use pin 2 as PWM in this config?
///
/// GSCLK output on Due pin 61 (analog 7):
///   GSCLK_TC = TC0_CH1
///   BLANK_TC = TC0_CH2
///
/// GSCLK output on Due pin 5 (digital 5):
///   GSCLK_TC = TC2_CH0
///   BLANK_TC = TC2_CH1  // can't use pin 3 as PWM in this config?
///
/// GSCLK output on Due pin 5 (digital 5):
///   GSCLK_TC = TC2_CH0
///   BLANK_TC = TC2_CH2  // can't use pin 11 as PWM in this config?
///
/// GSCLK output on Due pin 3 (digital 3):
///   GSCLK_TC = TC2_CH1
///   BLANK_TC = TC2_CH0  // can't use pin 5 as PWM in this config?
///
/// GSCLK output on Due pin 3 (digital 3):
///   GSCLK_TC = TC2_CH1
///   BLANK_TC = TC2_CH2  // can't use pin 11 as PWM in this config?
///
/// GSCLK output on Due pin 11 (digital 11):
///   GSCLK_TC = TC2_CH2
///   BLANK_TC = TC2_CH0  // can't use pin 5 as PWM in this config?
///
/// GSCLK output on Due pin 11 (digital 11):
///   GSCLK_TC = TC2_CH2
///   BLANK_TC = TC2_CH1  // can't use pin 3 as PWM in this config?
///


////////////////////////////////////////////////////////////////////////////////

// # of TLC5940 chips daisy-chained:
#define NUM_TLCS 1


// Timer counter channel which will be used to generate the GSCLK signal.  Must 
// be one of the TC*_CH* values.
#define GSCLK_TC TC2_CH2


// Timer counter channel which will be used to generate the BLANK signal.  
// Because the output of the GSCLK will be used as the source clock for this 
// timer, both the GSCLK_TC and BLANK_TC must be differing channels on the same 
// timer counter (e.g. TC2_CH1 and TC2_CH2).  Must be one of the TC*_CH* values.
#define BLANK_TC TC2_CH0


// USART that should be used for the synchronous serial communications to/from 
// the TLC5940.  Must be one of the TLC5940_USART* values.  
// Note that this library uses a USART instead of an SPI unit even though the 
// TLC5940 communications are very much like SPI.  I did this because I'm using 
// this library for a board I designed that fits the Arduino Due like a shield 
// and I didn't have room in the center to make use of the Due's SPI header.
#define TLC_USART TLC5940_USART0


// The following pin outputs the BLANK signal.
#define BLANK_PIN 9


// This pin output the XLAT signal (to latch in new data).
#define XLAT_PIN  10


// This pin will drive the VPRG pin.  It's low when sending grayscale data and 
// high when sending dot-correction data.
#define VPRG_PIN  6


// Input pin for the XERR signal from the TLC5940.  I might not end up 
// implementing code to make use of this.
#define XERR_PIN  2


// The following array specifies the initial dot-correction data for the 
// TLC5940 output channels.  0 disables all output on that channel, 63 means
// that maximum output is allowed.  There should be as many entries as there
// are TLC5940 output channels (i.e. NUM_TLCS * 16).  The first entry is 
// channel 0.

#define INITIAL_DCDATA {63, 63, 63, 63, \
                        63, 63, 63, 63, \
                        63, 63, 63, 63, \
                        63, 63, 63, 63}


////////////////////////////////////////////////////////////////////////////////
/// End of Code
