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

#include "Tlc5940.h"
#include <CapacitiveSensor.h>

#define FORWARD HIGH
#define BACK LOW

int l = 0;
int k = 0;
int direction = 1;
int setPoint = 0;
int posFader = 0;
int error = 0;
int error_ant = 0;
int Kp = 0.30, Ki = 0.20, Kd = 30;
int salida = 0;
const int setPin = 12;
const int posPin = 11;
const int canalMotor = 14;
const int forwardPin = 7;
const int backPin = 6;

CapacitiveSensor   cs_4_2 = CapacitiveSensor(4,2);        // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired

void setup()
{
  // INICIALIZO LA LIBRERIA CAP_SENSE
  cs_4_2.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example
  // INICIALIZO LA COMUNICACIÓN SERIE
   Serial.begin(9600);
   
  // INICIALIZO LA LIBRERIA PARA EL MANEJO DE TLC5940
  Tlc.init();
  
  pinMode(forwardPin,OUTPUT);
  pinMode(backPin,OUTPUT);  
  digitalWrite(forwardPin,LOW);
  digitalWrite(backPin,LOW);
  Fader_a_Cero();
}



void loop()
{
    setPoint = analogRead(setPin);   //LEO EL POTE
    if (setPoint>900) setPoint = 900; //SATURO EN MAXIMA POSICIÓN DEL FADER
    posFader = analogRead(posPin);   //LEO POSICIÓN del FADER
  
    if (setPoint>posFader){
      error_ant = 0;
      error = setPoint - posFader;
      salida = Kp*error + Ki*(error_ant+error)/2 + Kd*(error-error_ant);
      while (salida>30){
        Serial.println("ADELANTE");
        digitalWrite(forwardPin,HIGH);
        setPoint = analogRead(setPin);
        posFader = analogRead(posPin);         
        error_ant = error;
        error = setPoint - posFader;        
        salida = Kp*error + Ki*(error_ant+error)/2 + Kd*(error-error_ant);        
      }
      digitalWrite(forwardPin,LOW);
    }
    else if (setPoint<posFader){
      error_ant = 0;
      error = posFader - setPoint;
      salida = Kp*error + Ki*(error_ant+error)/2 + Kd*(error-error_ant);
      while (salida>30){
        Serial.println("ATRAS\n");
        Serial.println("Salida PID: ");
        Serial.println(salida);Serial.println("\n");
        Serial.println("Error leido: ");
        Serial.println(salida);Serial.println("\n\n");        
        
        digitalWrite(backPin,HIGH);
        setPoint = analogRead(setPin);
        posFader = analogRead(posPin);         
        error_ant = error;        
        error = posFader - setPoint;
        salida = Kp*error + Ki*(error_ant+error)/2 + Kd*(error-error_ant);
      }
      digitalWrite(backPin,LOW);
    }


    /* Tlc.clear() sets all the grayscale values to zero, but does not send
       them to the TLCs.  To actually send the data, call Tlc.update()
    Tlc.clear();
    long total1 =  cs_4_2.capacitiveSensor(20);
    Serial.println(total1);
    /* Tlc.set(channel (0-15), value (0-4095)) sets the grayscale value for
       one channel (15 is OUT15 on the first TLC, if multiple TLCs are daisy-
       chained, then channel = 16 would be OUT0 of the second TLC, etc.).

       value goes from off (0) to always on (4095).

       Like Tlc.clear(), this function only sets up the data, Tlc.update()
       will send the data. */
    /*if (total1>200){
      
      digitalWrite(forwardPin,LOW);
      Serial.println("ACTIVADO");
      Tlc.set(canalMotor,4000);
      Tlc.update();      
    }
    else{
      digitalWrite(forwardPin,LOW);
      Serial.println("DESACTIVADO");
      Tlc.set(canalMotor,0);
      Tlc.update();      

    }
    //}
//    delay(10);
    /* Tlc.update() sends the data to the TLCs.  This is when the LEDs will
       actually change. */


}

void Fader_a_Cero(void){
  posFader = analogRead(posPin);
  digitalWrite(forwardPin,LOW);
  digitalWrite(backPin,HIGH);
//  Tlc.set(canalMotor-1,4095);
//  Tlc.set(canalMotor,0);
//  Tlc.update();
  while(posFader>20){   
    posFader = analogRead(posPin);
    Serial.println(posFader);
  }
  digitalWrite(forwardPin,LOW);
  digitalWrite(backPin,LOW);
//  Tlc.set(canalMotor-1,0);
//  Tlc.set(canalMotor,0);
//  Tlc.update();
  Serial.println("Posicion final del fader: ");
  Serial.println(posFader);
}
