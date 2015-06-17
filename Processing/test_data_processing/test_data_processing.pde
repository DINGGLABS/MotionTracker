/* imports */
import processing.serial.*;

/* defines */
int X = 600;
int Y = 600;

int REF_H = 400;
int REF_B = 150;

int NUMBER_OF_SENSOR_DATA = 11;  // dt, ax, ay, az, Bx, By, Bz, Gxy, Gxz, Gyz (in this order!)
float GRAVITY = 9.81;

float FILTER_CONSTANT = 0.5;

/* communication */
int BAUDRATE = 112600;
int SERIAL_PORT = 5;
boolean firstContact = false;
String s = "knock";

//boolean ROTATE = true;                      // rotate data to fit them to the room coordinate system
//boolean G_ROTATION = true;                  // rotate by the g-error
boolean REMOVE_OFFSETS = true;               // remove offsets
//boolean EXP_ERROR_CORRECTION = true;        // remove exponential errors
//boolean TRAPEZ = false;                     // alternative integration calculation
//
//boolean SHOW_RAW_DATA = false;              // shows raw data
//boolean SHOW_ROTATED_DATA = false;          // shows data after the rotations
boolean SHOW_OFFSETS = true;                 // shows data after the calibration
//boolean SHOW_EXP_ERROR_CORRECTION = false;  // shows velocity vectors after the exponential error corrections
boolean SHOW_VECTORS = true;                // shows id, time, acceleration-, velocity- and trail-vectors
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

ArrayList<float[]> data = new ArrayList();                // global generic array list
ArrayList<float[]> offsets = new ArrayList();             // global generic array list
ArrayList<float[]> orientationVector = new ArrayList();   // global generic array list
ArrayList<float[]> accelerationVector = new ArrayList();  // global generic array list
ArrayList<float[]> velocityVector = new ArrayList();      // global generic array list
ArrayList<float[]> trailVector = new ArrayList();         // global generic array list

boolean calibration = true;
int NUMBER_OF_DATA_TO_CALIBRATE = 50;
int cCnt = 0;  // calibration counter

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
  
  //drawAccelerationVectors();
  //drawVelocityVectors();
  //drawTrailVectors();
    
  popMatrix();
}

void serialEvent( Serial myPort)
{
  /* put the incoming data into a String - */
  String stringVal = myPort.readStringUntil('\n');  //the '\n' is our end delimiter indicating the end of a complete packet
  
  /* make sure our data isn't empty before continuing */
  if (stringVal != null)
  {
    /* trim whitespace and formatting characters (like carriage return) */
    stringVal = trim(stringVal);  //blup zwingend nötig für Erkennung von "knock"
    //println(stringVal);
      
    if (firstContact == false)
    {
      /* look for our "knock" string to start the handshake */
      if (stringVal.equals(s))
      {
        /* if it's there, clear the buffer, and send a request for data */
        myPort.clear();
        firstContact = true;
        myPort.write("let's go!");
        println("contact established!");
        
        data.add(0, null);                 // init data array
        float zeros[] = {0, 0, 0};
        orientationVector.add(0, zeros);   // init orientation vector
        accelerationVector.add(0, zeros);  // init acceleration vector
        velocityVector.add(0, zeros);      // init acceleration vector
        trailVector.add(0, zeros);         // init acceleration vector
      }
    }
    else
    { //if we've already established contact, keep getting and parsing data
      float da[] = (float(split(stringVal, ',')));  //parses the packet and places the stringValues into the data array
      
      /* check if data is not a number */
      if (!Float.isNaN(da[0]))
      {
        /* set data vector */
        data.set(0, da);    
        println(str(data.get(0)));
        
        /* check for calibration */
        if (calibration)
        {
          /* collect first few data */
          if (cCnt < NUMBER_OF_DATA_TO_CALIBRATE)
          {
            println("collect calibration data: ");
            data.add(cCnt+1, da);
            cCnt++;
          }
          else
          {
            float dt_os = 0;
            float ax_os = 0;
            float ay_os = 0;
            float az_os = 0;
            float wx_os = 0;
            float wy_os = 0;
            float wz_os = 0;
            
            /* get mean values (offsets) */
            for (int i = 1; i <= NUMBER_OF_DATA_TO_CALIBRATE; i++)
            {
              dt_os += data.get(i)[0];
              ax_os += data.get(i)[1]; ay_os += data.get(i)[2]; az_os += data.get(i)[3];
              wx_os += data.get(i)[7]; wy_os += data.get(i)[8]; wz_os += data.get(i)[9];
            }
            dt_os /= NUMBER_OF_DATA_TO_CALIBRATE;
            ax_os /= NUMBER_OF_DATA_TO_CALIBRATE; ay_os /= NUMBER_OF_DATA_TO_CALIBRATE; az_os /= NUMBER_OF_DATA_TO_CALIBRATE;
            wx_os /= NUMBER_OF_DATA_TO_CALIBRATE; wy_os /= NUMBER_OF_DATA_TO_CALIBRATE; wz_os /= NUMBER_OF_DATA_TO_CALIBRATE;
                        
            if (SHOW_OFFSETS)
            {
              print("offsets (g-vector): ");
              println(ax_os + ", " + ay_os + ", " + az_os);
              print("gyro offsets: ");
              println(wx_os + ", " + wy_os + ", " + wz_os);
            }
            
          /* calibration data processing: */
            /* get dt */
            dt_os /= 1000000;  // in s
            
            /* set offset acceleration vector */
            float a_os[] = {ax_os, ay_os, az_os};
            
            /* get offset orientations */
            float o_g_os[] = getOrientationGyro(dt_os, wx_os, wy_os, wz_os);
            float o_a_os[] = getOrientationAccel(ax_os, ay_os, az_os);
            
            /* fuse offset orientations */
            float o_os[] = fuseOrientations(FILTER_CONSTANT, o_g_os, o_a_os);
                        
            /* set global offset vectors */
            offsets.add(0, a_os);  // accelerations
            offsets.add(1, o_os);  // orientation
            
            calibration = false;
          }
        }
        else
        {
        /* data processing: */
          /* get dt */
          float dt = data.get(0)[0] / 1000000;  // in s
          
          /* get a */
          float ax = data.get(0)[1];
          float ay = data.get(0)[2];
          float az = data.get(0)[3];
          
          /* get w */
          float wx = data.get(0)[7];
          float wy = data.get(0)[8];
          float wz = data.get(0)[9];
          
          /* get orientations */
          float o_g[] = getOrientationGyro(dt, wx, wy, wz);
          float o_a[] = getOrientationAccel(ax, ay, az);
          
          /* fuse orientations */
          float o[] = fuseOrientations(FILTER_CONSTANT, o_g, o_a);
          
//          /* rotate orientations by the orientation offsets */
//          o = rotateArrayVector(o, offsets.get(1)[0], offsets.get(1)[1], offsets.get(1)[2]);  // (o, roll, pitch, heading)
          
          /* remove orientation offsets -> have to be called befor acceleration rotation! */
          o[0] -= offsets.get(1)[0];  // roll
          o[1] -= offsets.get(1)[1];  // pitch
          o[2] -= offsets.get(1)[2];  // heading
          
          if (SHOW_OFFSETS)
          {
            print("orientation offsets: ");
            println(offsets.get(1)[0]*180/PI + ", " + offsets.get(1)[1]*180/PI + ", " + offsets.get(1)[2]*180/PI);
            println("roll = " + o[0]*180/PI + " pitch = " + o[1]*180/PI + " heading = " + o[2]*180/PI);
          }
          
          /* update global orientation vector */
          orientationVector.set(0, o);
          
//          /* remove acceleration offsets (including g) -> have to be called befor acceleration rotation! */
//          ax -= offsets.get(0)[0];
//          ay -= offsets.get(0)[1];
//          az -= offsets.get(0)[2];
          
          if (SHOW_OFFSETS)
          {
            print("accel offsets: ");
            println(offsets.get(0)[0] + ", " + offsets.get(0)[1] + ", " + offsets.get(0)[2]);
          }
          
          /* rotate acceleration vector to fit them to the room coordinate system */
          float a[] = {ax, ay, az};
          a = rotateArrayVector(a, o[0], o[1], o[2]);  // (a, roll, pitch, heading)
          
          /* remove g offset */
          a[2] -= sqrt(offsets.get(0)[0]*offsets.get(0)[0] + offsets.get(0)[1]*offsets.get(0)[1] + offsets.get(0)[2]*offsets.get(0)[2]);
          
          /* update global acceleration vector */
          accelerationVector.set(0, a);
          
          /* get velocity vecotr */
          float v[] = getVelocityVector(dt);
          velocityVector.add(0, v);  // update global velocity vector
          
          /* get trail vector */
          float s[] = getTrailVector(dt); 
          trailVector.add(0, s);  // update global trail vector
          
          /* display data */
          showVectors();
        }
      }
    }
  }
}

//          /* remove offsets */
//          if(REMOVE_OFFSETS)
//          {
//            data.get(0)[1] -= offsets.get(0)[0];  // ax
//            data.get(0)[2] -= offsets.get(0)[1];  // ay
//            data.get(0)[3] -= offsets.get(0)[2];  // az
//            data.get(0)[7] -= offsets.get(0)[3];  // Gx
//            data.get(0)[8] -= offsets.get(0)[4];  // Gy
//            data.get(0)[9] -= offsets.get(0)[5];  // Gz
//            
//            if (SHOW_OFFSETS) println(str(data.get(0)));
//          }




/*------------------------------------------------------------------*/
/* integrate gyro values to get orientations in rad */
float[] getOrientationGyro(float dt, float wx, float wy, float wz)
{         
  // orientation_new = orientation_old + w * dt
  float roll_g    = orientationVector.get(0)[0] + wx * dt * PI/180;  // fPhi(wx)
  float pitch_g   = orientationVector.get(0)[1] + wy * dt * PI/180;  // fTheta(wy)
  float heading_g = orientationVector.get(0)[2] + wz * dt * PI/180;  // fPsi(wz)
  //println("roll_g = " + roll_g*180/PI + " pitch_g = " + pitch_g*180/PI + " heading_g = " + heading_g*180/PI);
  
  float o_g[] = {roll_g, pitch_g, heading_g};
  return o_g;
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* calculate orientations in rad from the acceleration vector */
float[] getOrientationAccel(float ax, float ay, float az)
{
  float roll_a    = atan2(ay, sqrt(ax*ax + az*az));  // get roll (yz-axis rotation) out of the acceleration values
  float pitch_a   = atan2(ax, sqrt(ay*ay + az*az));  // get pitch (xz-axis rotation) out of the acceleration values
  float heading_a = getHeading_a(roll_a, pitch_a);
  //println("roll_a = " + roll_a*180/PI + " pitch_a = " + pitch_a*180/PI + " heading_a = " + heading_a*180/PI);
  
  float o_a[] = {roll_a, pitch_a, heading_a};
  return o_a;
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* fuse orientations: complementary filter */
float[] fuseOrientations(float fc, float[] g, float[] a)  // the bigger fc the better the result but slower the system!
{
//        float w_magnitude = sqrt(data.get(0)[7]*data.get(0)[7] + data.get(0)[8]*data.get(0)[8] + data.get(0)[9]*data.get(0)[9]);
//        float fc = constrain(mapFloat(abs(w_magnitude), 0, 300, 0, 1), 0, 1);  // calculate fc on base of gyro magnitude, map and limit it to 0... 1
  float roll    = fc * g[0] + (1 - fc) * a[0];
  float pitch   = fc * g[1] + (1 - fc) * a[1];
  float heading = fc * g[2] + (1 - fc) * a[2];
  println("roll = " + roll*180/PI + " pitch = " + pitch*180/PI + " heading = " + heading*180/PI);
  
  /* update orientation vector */
  float o[] = {roll, pitch, heading};
  return o;
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/ 
///* get tilt compensated heading (xy-axis rotation) out of the magnetometer values */
//float getHeading_m(float roll, float pitch)
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

/* get tilt compensated heading (xy-axis rotation) out of the acceleration values */
float getHeading_a(float roll, float pitch)
{
  float headX, headY;
  
  float cos_roll = cos(roll);
  float sin_roll = 1  - (cos_roll * cos_roll);
  float cos_pitch = cos(pitch);
  float sin_pitch = 1  - (cos_pitch * cos_pitch);
  
  float ax = data.get(0)[1];
  float ay = data.get(0)[2];
  float az = data.get(0)[3];
  
  /* normalize acceleration values */
  float a_amount = sqrt(ax*ax + ay*ay + az*az);
  ax /= a_amount;
  ay /= a_amount;
  az /= a_amount;
  
  /* heading */
  headX = ax*cos_pitch + ay*sin_roll*sin_pitch + az*cos_roll*sin_pitch;  // tilt compensated X component
  headY = ay*cos_roll - az*sin_roll;                                     // tilt compensated Y component
  float heading = atan2(-headY, headX);
  
  /* declination correction (if supplied) */
//  if(abs(_declination) > 0.0)
//  {
//      heading = heading + _declination;
//      if (heading > M_PI) heading -= (2.0 * PI);  // angle normalization (-180 deg, 180 deg)
//      else if (heading < -M_PI) heading += (2.0 * PI);
//  }
  
  return heading;  // in rad
}
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/ 
/* integrate acceleration vector to get the velocity vector */
float[] getVelocityVector(float dt)
{
  // v_new = v_old + a * dt
  float vx = velocityVector.get(0)[0] + accelerationVector.get(0)[0] * dt;
  float vy = velocityVector.get(0)[1] + accelerationVector.get(0)[1] * dt;
  float vz = velocityVector.get(0)[2] + accelerationVector.get(0)[2] * dt;
  float v[] = {vx, vy, vz}; 
  return v;
}
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/ 
/* integrate velocity vector to get the trail vector */
float[] getTrailVector(float dt)
{
  // s_new = s_old + v * dt
  float sx = trailVector.get(0)[0] + velocityVector.get(0)[0] * dt;
  float sy = trailVector.get(0)[1] + velocityVector.get(0)[1] * dt;
  float sz = trailVector.get(0)[2] + velocityVector.get(0)[2] * dt;
  float s[] = {sx, sy, sz}; 
  return s;
}
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/   
void showVectors()
{
  if (SHOW_VECTORS)
  {
    println("show vectors:"); print("dt"); print("\t"); print("ax, ay, az");
    print("\t\t\t\t\t"); print("vx, vy, vz"); print("\t\t\t\t"); println("sx, sy, sz");
    
    print(str(data.get(0)[0]));
    print("\t");
    print(str(accelerationVector.get(0)));
    print("\t\t");
    print(str(velocityVector.get(0)));
    print("\t\t");
    println(str(trailVector.get(0)));
    println("roll = " + orientationVector.get(0)[0]*180/PI + " pitch = " + orientationVector.get(0)[1]*180/PI + " heading = " + orientationVector.get(0)[2]*180/PI);
    println("");
  }
}
/*------------------------------------------------------------------*/ 

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

///*------------------------------------------------------------------*/  
//void drawAccelerationVectors()
//{
//  for (int i = 1; i < numberOfRows; i++)
//  {
//    stroke(int(i * (255.0 / numberOfRows)), int(i * (255.0 / numberOfRows)), int(i * (255.0 / numberOfRows)));
//    float last_dsx = accelerationVectors.get(i-1)[0] * ma;
//    float last_dsy = accelerationVectors.get(i-1)[1] * ma;
//    float last_dsz = accelerationVectors.get(i-1)[2] * ma;
//    float new_dsx = accelerationVectors.get(i)[0] * ma;
//    float new_dsy = accelerationVectors.get(i)[1] * ma;
//    float new_dsz = accelerationVectors.get(i)[2] * ma;
//    
////    line(last_dsx, last_dsy, new_dsx, new_dsy);             // 2D (x and y)
////    line(last_dsx, last_dsz, new_dsx, new_dsz);             // 2D (x and z)
////    line(last_dsy, last_dsz, new_dsy, new_dsz);             // 2D (y and z)
//    line(last_dsx, last_dsy, last_dsz, new_dsx, new_dsy, new_dsz);  // 3D
//
////    /* draw points */
////    if (i%5 == 0)
////    {
////      pushMatrix();
////      translate(new_dsx, new_dsy, new_dsz);
////      sphere(1);
////      popMatrix();
////    }
//  }
//}
///*------------------------------------------------------------------*/
//
///*------------------------------------------------------------------*/  
//void drawVelocityVectors()
//{
//  for (int i = 1; i < numberOfRows; i++)
//  {
//    stroke(255, 50, int(i * (255.0 / numberOfRows)));
//    float last_dsx = velocityVectors.get(i-1)[0] * mv;
//    float last_dsy = velocityVectors.get(i-1)[1] * mv;
//    float last_dsz = velocityVectors.get(i-1)[2] * mv;
//    float new_dsx = velocityVectors.get(i)[0] * mv;
//    float new_dsy = velocityVectors.get(i)[1] * mv;
//    float new_dsz = velocityVectors.get(i)[2] * mv;
//    
////    line(last_dsx, last_dsy, new_dsx, new_dsy);             // 2D (x and y)
////    line(last_dsx, last_dsz, new_dsx, new_dsz);             // 2D (x and z)
////    line(last_dsy, last_dsz, new_dsy, new_dsz);             // 2D (y and z)
//    line(last_dsx, last_dsy, last_dsz, new_dsx, new_dsy, new_dsz);  // 3D
//
////    /* draw points */
////    if (i%5 == 0)
////    {
////      pushMatrix();
////      translate(new_dsx, new_dsy, new_dsz);
////      sphere(1);
////      popMatrix();
////    }
//  }
//}
///*------------------------------------------------------------------*/
//
///*------------------------------------------------------------------*/  
//void drawTrailVectors()
//{
//  for (int i = 1; i < numberOfRows; i++)
//  {
//    stroke(0, 255, int(i * (255.0 / numberOfRows)));
//    float last_dsx = trailVectors.get(i-1)[0] * ms;
//    float last_dsy = trailVectors.get(i-1)[1] * ms;
//    float last_dsz = trailVectors.get(i-1)[2] * ms;
//    float new_dsx = trailVectors.get(i)[0] * ms;
//    float new_dsy = trailVectors.get(i)[1] * ms;
//    float new_dsz = trailVectors.get(i)[2] * ms;
//    
////    line(last_dsx, last_dsy, new_dsx, new_dsy);             // 2D (x and y)
////    line(last_dsx, last_dsz, new_dsx, new_dsz);             // 2D (x and z)
////    line(last_dsy, last_dsz, new_dsy, new_dsz);             // 2D (y and z)
//    line(last_dsx, last_dsy, last_dsz, new_dsx, new_dsy, new_dsz);  // 3D
//
////    /* draw points */
////    if (i%5 == 0)
////    {
////      pushMatrix();
////      translate(new_dsx, new_dsy, new_dsz);
////      sphere(1);
////      popMatrix();
////    }
//  }
//}
///*------------------------------------------------------------------*/

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
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
/*------------------------------------------------------------------*/
