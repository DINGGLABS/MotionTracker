/* imports */
import processing.serial.*;

/* defines */
int X = 600;
int Y = 600;

int REF_H = 400;
int REF_B = 150;

int NUMBER_OF_SENSOR_DATA = 11;  // dt, ax, ay, az, Bx, By, Bz, Gxy, Gxz, Gyz (in this order!)

/* communication */
int BAUDRATE = 112600;
int SERIAL_PORT = 5;
boolean firstContact = false;
String s = "knock";

//boolean ROTATE = true;                      // rotate data to fit them to the room coordinate system
//boolean G_ROTATION = true;                  // rotate by the g-error
//boolean ACCEL_OFFSETS = true;               // remove g-shares
//boolean ANGLE_OFFSETS = false;              // remove angle offsets
//boolean EXP_ERROR_CORRECTION = true;        // remove exponential errors
//boolean TRAPEZ = false;                     // alternative integration calculation
//
//boolean SHOW_RAW_DATA = false;              // shows raw data
//boolean SHOW_ROTATED_DATA = false;          // shows data after the rotations
//boolean SHOW_CALIBRATED_DATA = false;       // shows data after the calibration
//boolean SHOW_EXP_ERROR_CORRECTION = false;  // shows velocity vectors after the exponential error corrections
//boolean SHOW_VECTORS = true;                // shows id, time, acceleration-, velocity- and trail-vectors
//
///* drawing multiplicators */
//int ma = 10;
//int mv = 10;
//int ms = 10;

/* global variables */
int mX, mY;    // mouse coordinates
int scaleFactor;
float translateZ;

Serial myPort;

ArrayList<float[]> data = new ArrayList();         // global generic array list
ArrayList<float[]> orientation = new ArrayList();  // global generic array list

/* setup */
void setup()
{
  /* setup window */
  //size(X, Y);
  size(X, Y, P3D);
  //background(255, 255, 255);
  textSize(20);
  stroke(255);
  scaleFactor = 1;
  
  /* init serial port */
  myPort = new Serial(this, Serial.list()[SERIAL_PORT], BAUDRATE);
}

/* loop */
void draw()
{ 
  //draw2DDiagramAxes();
  draw3DDiagramAxes();
  
  
  
  
    
  popMatrix();
}

void serialEvent( Serial myPort)
{
  //put the incoming data into a String - 
  //the '\n' is our end delimiter indicating the end of a complete packet
  String stringVal = myPort.readStringUntil('\n');
  //make sure our data isn't empty before continuing
  if (stringVal != null)
  {
    //trim whitespace and formatting characters (like carriage return)
    stringVal = trim(stringVal);  //blup zwingend nötig für Erkennung von "knock"
    //println(stringVal);
      
    //look for our "knock" string to start the handshake
    //if it's there, clear the buffer, and send a request for data
    if (firstContact == false)
    {
      if (stringVal.equals(s))
      {
        myPort.clear();
        firstContact = true;
        myPort.write("let's go!");
        println("contact established!");
        
        data.add(0, null);         // init data array
        float zeros[] = {0, 0, 0};
        orientation.add(0, zeros);  // init orientation_old array
      }
    }
    else
    { //if we've already established contact, keep getting and parsing data
      float a[] = (float(split(stringVal, ',')));  //parses the packet and places the stringValues into the data array
      
      if (!Float.isNaN(a[0]))
      {
        data.set(0, a);    
        println(str(data.get(0)));
      
      /* data processing: */
        float dt = data.get(0)[0] / 1000000;  // in s
        
        /* integrate gyro values */
        // orientation_new = orientation_old + w * dt
        float roll    = orientation.get(0)[0] + data.get(0)[7] * dt;  // fPhi(wx)
        float pitch   = orientation.get(0)[1] + data.get(0)[8] * dt;  // fTheta(wy)
        float heading = orientation.get(0)[2] + data.get(0)[9] * dt;  // fPsi(wz)
        float b[] = {roll, pitch, heading};
        orientation.set(0, b);

        //println("roll = " + orientation.get(0)[0] + " pitch = " + orientation.get(0)[1] + " heading = " + orientation.get(0)[2]);
        
        
        
//        /* get orientation */
//        float roll = getRoll();
//        float pitch = getPitch();
//        float heading = getHeading(roll, pitch);
//        
//        println(roll*180/PI);
//        println(pitch*180/PI);
//        println(heading*180/PI);      
      
      
      
      
      
      }
    }
  }
}

/* get roll (yz-axis rotation) out of the acceleration values */
float getRoll()
{
  float ax = data.get(0)[1];
  float ay = data.get(0)[2];
  float az = data.get(0)[3];
  
  return atan2(ay, sqrt(ax*ax + az*az));
}

/* get pitch (xz-axis rotation) out of the acceleration values */
float getPitch()
{
  float ax = data.get(0)[1];
  float ay = data.get(0)[2];
  float az = data.get(0)[3];
  
  return atan2(ax, sqrt(ay*ay + az*az));  // in rad
}

///* get heading (xy-axis rotation) out of the magnetometer values */
//float getHeading(float roll, float pitch)
//{
//  float headX, headY;
//  
//  float cos_roll = cos(roll);
//  float sin_roll = 1  - (cos_roll * cos_roll);
//  float cos_pitch = cos(pitch);
//  float sin_pitch = 1  - (cos_pitch * cos_pitch);
//  
//  float Bx = data.get(0)[4];
//  float By = data.get(0)[5];
//  float Bz = data.get(0)[6];
//  
//  /* normalize magnetometer values */
//  float B_amount = sqrt(Bx*Bx + By*By + Bz*Bz);
//  Bx /= B_amount;
//  By /= B_amount;
//  Bz /= B_amount;
//  
//  /* magnetic heading */
//  headX = Bx*cos_pitch + By*sin_roll*sin_pitch + Bz*cos_roll*sin_pitch;  // tilt compensated magnetic field X component
//  headY = By*cos_roll - Bz*sin_roll;                                     // tilt compensated magnetic field Y component
//  float heading = atan2(-headY, headX);
//
////  /* declination correction (if supplied) */
////  if( fabs(_declination) > 0.0 )
////  {
////      heading = heading + _declination;
////      if (heading > M_PI) heading -= (2.0 * PI);  // angle normalization (-180 deg, 180 deg)
////      else if (heading < -M_PI) heading += (2.0 * PI);
////  }
//  
//  return heading;  // in rad
//}  

/* get heading (xy-axis rotation) out of the acceleration- and gyroscope values */
float getHeading(float roll, float pitch)
{
  float headX, headY;
  
  float cos_roll = cos(roll);
  float sin_roll = 1  - (cos_roll * cos_roll);
  float cos_pitch = cos(pitch);
  float sin_pitch = 1  - (cos_pitch * cos_pitch);
  
  float ax = data.get(0)[4];
  float ay = data.get(0)[5];
  float az = data.get(0)[6];
  
  /* normalize acceleration values */
  float a_amount = sqrt(ax*ax + ay*ay + az*az);
  ax /= a_amount;
  ay /= a_amount;
  az /= a_amount;
  
  /* magnetic heading */
  headX = ax*cos_pitch + ay*sin_roll*sin_pitch + az*cos_roll*sin_pitch;  // tilt compensated magnetic field X component
  headY = ay*cos_roll - az*sin_roll;                                     // tilt compensated magnetic field Y component
  float heading = atan2(-headY, headX);
  
//  /* declination correction (if supplied) */
//  if( fabs(_declination) > 0.0 )
//  {
//      heading = heading + _declination;
//      if (heading > M_PI) heading -= (2.0 * PI);  // angle normalization (-180 deg, 180 deg)
//      else if (heading < -M_PI) heading += (2.0 * PI);
//  }
  
  return heading;  // in rad
}  

/*------------------------------------------------------------------*/ 


// loop functions:
/*------------------------------------------------------------------*/  
void draw2DDiagramAxes()
{
  background(0);
  translate(width/4, height/4, -height/2);
  
  pushMatrix();
  translate(0, 0, translateZ);
  scale(scaleFactor);
  
  if(mousePressed)
  {
    mX = mouseY;
    mY = mouseX;
  }
  
  rotateX(map(mX, 0, height, -PI, PI));
  rotateY(map(mY, 0, height, -PI, PI));
  
  draw2DAxes(X, 30);
}
/*------------------------------------------------------------------*/  
  
/*------------------------------------------------------------------*/  
void draw3DDiagramAxes()
{
  background(0);
  translate(width/2, height/2, -height/2);
  
  pushMatrix();
  translate(0, 0, translateZ);
  scale(scaleFactor);
  
  if(mousePressed)
  {
    mX = mouseY;
    mY = mouseX;
  }
  
  rotateX(map(mX, 0, height, -PI, PI));
  rotateY(map(mY, 0, height, -PI, PI));
  
  draw3DAxes(X/3*2, 30);
}
/*------------------------------------------------------------------*/  


// auxiliary functions:
/*------------------------------------------------------------------*/
void mouseWheel(MouseEvent e)
{
  translateZ -= e.getCount() * 5;
  scaleFactor += e.getCount() / 100;
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
void draw2DAxes(int l, int textOffset)
{
  int tl = l + textOffset;
  fill(255, 255, 255);
  stroke(255, 0, 0);  // red
  line(-l, 0, 0, l, 0, 0);
  text("+t", l, 0, 0);
  text("-t", -tl, 0, 0);
  
  stroke(0, 255, 0);  // green
  line(0, -l, 0, 0, l, 0);
  text("+var", 0, tl, 0);
  text("-var", 0, -l, 0);
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
void draw3DAxes(int l, int textOffset)
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
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
float[] rotateArrayVector(float vec[], float yzAngle, float xzAngle, float xyAngle)
{
  /* collect current data */
  float x = vec[0];
  float y = vec[1];
  float z = vec[2];
  float x_new1, y_new1, z_new1, x_new2, y_new2, z_new2;
  float roll = yzAngle * PI/180;     // x-rotation angle
  float pitch = xzAngle * PI/180;    // y-rotation angle
  float heading = xyAngle * PI/180;  // z-rotation angle
          
  //println(str(data.get(i)));
  
  /* x-axis rotation: */
  y_new1 = y * cos(roll) - z * sin(roll);
  z_new1 = y * sin(roll) + z * cos(roll);
  
  /* y-axis rotation: */
  x_new1 = x * cos(pitch) + z_new1 * sin(pitch);
  z_new2 = -x * sin(pitch) + z_new1 * cos(pitch);
  
  /* z-axis rotation: */
  x_new2 = x_new1 * cos(heading) - y_new1 * sin(heading);
  y_new2 = x_new1 * sin(heading) + y_new1 * cos(heading);
  
  float a[] = {x_new2, y_new2, z_new2};
  return a;
  
  //println(str(accelerationVectors.get(i)));
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
double nthRoot(int n, float root)
{ 
  //return Math.pow(Math.E, Math.log(n)/root);    // returend wrong results!
  return pow(root, 1.0/n);
} 
/*------------------------------------------------------------------*/
