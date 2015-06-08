/* imports */
import processing.serial.*;

/* defines */
int X = 500;
int Y = 500;

int BAUDRATE = 115200;
int SERIAL_PORT = 5;
String s = "knock";

/* global variables */
int mX, mY;

Serial myPort;
String stringVal;
String dataLines[];
boolean firstContact = false;

long t;
volatile float ax, ay, az, ax_alt, ay_alt, az_alt, dax, day, daz;

/* setup */
void setup()
{
  size(X, Y, P3D);
  background(0);
  textSize(20);
  stroke(255);
  
  /* serial interface */
  myPort = new Serial(this, Serial.list()[SERIAL_PORT], BAUDRATE);
  
  
}

/* loop */
void draw()
{
/* diagram */
  background(0);
  translate(width/2, height/2, -height/2);
  
  if(mousePressed)
  {
    mX = mouseY;
    mY = mouseX;
  }
  
  rotateX(map(mX, 0, height, -PI, PI));
  rotateY(map(mY, 0, height, -PI, PI));
  
  drawAxes(300, 30);
    
/* draw sphere */
  dax = abs(ax - ax_alt);
  day = abs(ay - ay_alt);
  daz = abs(az - az_alt);
  noStroke();
  fill(200, 100, 0);
  lights();
  sphere((dax + day + daz) * 3);
  ax_alt = ax;
  ay_alt = ay;
  az_alt = az;
  
//  delay(50);
}
 
void drawAxes(int l, int textOffset)
{
  int tl = l + textOffset;
  fill(255, 255, 255);
  stroke(255, 0, 0);  // red
  line(-l, 0, 0, l, 0, 0);
  text("+x", l, 0, 0);
  text("-x", -tl, 0, 0);
  
  stroke(0, 255, 0);  // green
  line(0, -l, 0, 0, l, 0);
  text("+y", 0, tl, 0);
  text("-y", 0, -l, 0);
  
  stroke(0, 0, 255);  //blue
  line(0, 0, -l, 0, 0, l);
  text("+z", 0, 0, tl);
  text("-z", 0, 0, -l);
}

void serialEvent( Serial myPort)
{
  /* put the incoming data into a string */
  stringVal = myPort.readStringUntil('\n');  //the '\n' is our end delimiter indicating the end of a complete packet
    
  //make sure our data isn't empty before continuing
  if (stringVal != null)
  {
    //trim whitespace and formatting characters (like carriage return)
    stringVal = trim(stringVal);  //blup zwingend nötig für Erkennung von "knock"
    println(stringVal);
      
    //look for our "knock" string to start the handshake
    //if it's there, clear the buffer, and send a request for data
    if (firstContact == false)
    {
      if (stringVal.equals(s))
      {
        myPort.clear();
        firstContact = true;
        myPort.write("send me some data now!");  // uC waits until data have been sent
        println("contact established!");
      }
    }
    else
    { //if we've already established contact, keep getting and parsing data
      float sensorVals[] = float(split(stringVal, ','));  //parses the packet from Arduino and places the stringValues into the sensorVals array
      //println(sensorVals.length);
      //println(str(sensorVals));

      if (sensorVals.length >= 4)
      {
        t = (long)sensorVals[0];
        ax = sensorVals[1];
        ay = sensorVals[2];
        az = sensorVals[3];
      }
    }
  }
}
