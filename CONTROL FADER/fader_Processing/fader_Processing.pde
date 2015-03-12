
import oscP5.*;
import netP5.*;

import processing.serial.*;

Serial port;

OscP5 oscP5;
//NetAddress brodcast = new NetAddress("192.168.166.117", 8000);
NetAddress broadcast1 = new NetAddress("127.0.0.1", 8000);

int posFader;

void setup()
{
  size(1000, 200, P2D);
  oscP5 = new OscP5(this, 8001);
  
  String portName = "COM1";
  port = new Serial(this, portName, 9600);
  //notesSetup();
  frameRate(240);
}

boolean updateFader;

String inString = "";

int valA;

void draw()
{
     while (port.available() > 0)
    {
      inString = port.readStringUntil(13);
      if (inString == null)
        {
        }
       else if (inString != null )
        {
          println(inString);
          String sensor2 = trim(inString);
          valA = Integer.parseInt(sensor2);
        }  
    }     
  // create an osc message
  OscMessage osc_val1 = new OscMessage("/test");
  
 // osc_val1.add("A: "); // add an int to the osc message
  osc_val1.add(valA); // add an int to the osc message

  // send the message
  oscP5.send(osc_val1, broadcast1);
  
  /*if(updateFader){
    
  } */ 
}

void oscEvent(OscMessage theOscMessage)
{
  // get the first value as an integer
  posFader = theOscMessage.get(0).intValue();
  updateFader = true;

  String send = map(posFader,0,512,0,1023) + "\n";
  port.write(send);
  updateFader = false;
}

