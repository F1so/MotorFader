#include <CapacitiveSensorDue.h>

/*
 * CapitiveSense Library Demo Sketch
 * Paul Badger 2008
 * Uses a high value resistor e.g. 10M between send pin and receive pin
 * Resistor effects sensitivity, experiment with values, 50K - 50M. Larger
 * resistor values yield larger sensor values.
 * Receive pin is the sensor pin - try different amounts of foil/metal on this
 * pin.
 */

CapacitiveSensorDue cs_4_2 = CapacitiveSensorDue(44,45);	// 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired

void setup()					
{
        Serial.begin(9600);
        cs_4_2.setTimeout(100);
        cs_4_2.calibrate();
}

void loop()					
{	
	long total1 = cs_4_2.read(50);

//	Serial.print(millis() - start);	// check on performance in milliseconds
//	Serial.print("\t");				// tab character for debug windown spacing

	Serial.println(total1);			// print sensor output 1
}
