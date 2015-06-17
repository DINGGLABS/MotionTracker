/* imports */
import processing.serial.*;

/* defines */
int X = 600;
int Y = 600;

int REF_H = 400;
int REF_B = 150;

int NUMBER_OF_SENSOR_DATA = 11;  // dt, ax, ay, az, Bx, By, Bz, Gxy, Gxz, Gyz, T (in this order!)
//float GRAVITY = 9.81;

float FILTER_CONSTANT = 0.5;
float ACCEL_THRESHOLD = 0.5;  // threshold in m/s^2 to observe before integrating the accelerations

/* communication */
int BAUDRATE = 112600;
int SERIAL_PORT = 5;
boolean firstContact = false;
String s = "knock";  // handshacke key

/* switches */
boolean CHANGING_FILTER_CONSTANT = false;   // recalculate filter constant on base of current gyro magnitude
boolean USE_MAGNETOMETER = false;           // uses the magnetometer to calculate the heading instead of the acceleration vector
boolean ROTATE = true;                      // rotate data to fit them to the room coordinate system
boolean REMOVE_G_VECTOR = true;             // remove g-vector
boolean REMOVE_ORIENTATION_OFFSET = true;   // remove orientation offsets -> ATTENTION: only allowed when calibrated on a plane ground!
//boolean EXP_ERROR_CORRECTION = true;        // remove exponential velocity errors

boolean SHOW_RAW_DATA = true;               // shows raw data 
boolean SHOW_FILTERED_ORIENTATION = true;   // shows the orientation before removing the orientation offset
//boolean SHOW_EXP_ERROR_CORRECTION = false;  // shows velocity vectors after the exponential error corrections
boolean SHOW_VECTORS = true;                // shows the time-delta, acceleration-, velocity-, trail- and orientation vectors

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
ArrayList<float[]> calibrations = new ArrayList();        // global generic array list
ArrayList<float[]> orientationVector = new ArrayList();   // global generic array list
ArrayList<float[]> accelerationVector = new ArrayList();  // global generic array list
ArrayList<float[]> velocityVector = new ArrayList();      // global generic array list
ArrayList<float[]> trailVector = new ArrayList();         // global generic array list

/* calibration variables */
boolean calibrationFlag = true;
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
  
  /* init data array and vectors */
  data.add(0, null);                 
  float zeros[] = {0, 0, 0};
  calibrations.add(0, zeros);
  calibrations.add(1, zeros);
  orientationVector.add(0, zeros);   // init orientation vector
  accelerationVector.add(0, zeros);  // init acceleration vector
  velocityVector.add(0, zeros);      // init acceleration vector
  trailVector.add(0, zeros);         // init acceleration vector
}

/* loop */
void draw()
{ 
  //draw2DDiagramAxes();
  draw3DDiagramAxes();
  
  //drawAccelerationVectors();
  //drawVelocityVectors();
  //drawTrailVectors();
  
  drawFloatingObject();
    
  popMatrix();
}

void serialEvent( Serial myPort)
{
  /* put the incoming data into a string */
  String stringVal = myPort.readStringUntil('\n');  //the '\n' is our end delimiter indicating the end of a complete packet
  
  /* make sure our data isn't empty before continuing */
  if (stringVal != null)
  {
    /* trim whitespace and formatting characters (like carriage return) */
    stringVal = trim(stringVal);  // necessary to realize the handshacke key
    //println(stringVal);
      
    if (firstContact == false)
    {
      /* look for our "knock" string to start the handshake */
      if (stringVal.equals(s))
      {
        /* if it's there, clear the buffer and send a request for data */
        myPort.clear();
        firstContact = true;
        myPort.write("let's go!");  // data request
        println("contact established!");
      }
    }
    else
    { 
      /* if we've already established contact, keep getting and parsing data */
      float da[] = (float(split(stringVal, ',')));  //parses the packet and places the stringValues into the data array
      
      /* check if data is a number */
      if (!Float.isNaN(da[0]))
      {
        /* set data vector */
        data.set(0, da);    
        if (SHOW_RAW_DATA) println(str(data.get(0)));
        
        /* check calibration flag */
        if (calibrationFlag)
        {
          /* collect first few data for calibration */
          if (cCnt < NUMBER_OF_DATA_TO_CALIBRATE)
          {
            println("collect calibration data: ");
            data.add(cCnt+1, da);
            cCnt++;
          }
          else
          {
            float dt_c = 0;
            float ax_c = 0;
            float ay_c = 0;
            float az_c = 0;
            float wx_c = 0;
            float wy_c = 0;
            float wz_c = 0;
            
            /* get mean values (calibration) */
            for (int i = 0; i <= NUMBER_OF_DATA_TO_CALIBRATE; i++)
            {
              dt_c += data.get(i)[0];
              ax_c += data.get(i)[1]; ay_c += data.get(i)[2]; az_c += data.get(i)[3];
              wx_c += data.get(i)[7]; wy_c += data.get(i)[8]; wz_c += data.get(i)[9];
            }
            dt_c /= NUMBER_OF_DATA_TO_CALIBRATE;
            ax_c /= NUMBER_OF_DATA_TO_CALIBRATE; ay_c /= NUMBER_OF_DATA_TO_CALIBRATE; az_c /= NUMBER_OF_DATA_TO_CALIBRATE;
            wx_c /= NUMBER_OF_DATA_TO_CALIBRATE; wy_c /= NUMBER_OF_DATA_TO_CALIBRATE; wz_c /= NUMBER_OF_DATA_TO_CALIBRATE;
                        
                                    
          /* calibration data processing: */
            /* get dt */
            dt_c /= 1000000;  // in s
            
            /* set calibration acceleration vector */
            float a_c[] = {ax_c, ay_c, az_c};
            
            /* get calibration orientations */
            float o_g_c[] = getOrientationGyro(dt_c, wx_c, wy_c, wz_c);
            float o_a_c[] = getOrientationAccel(ax_c, ay_c, az_c);
                  
            /* fuse calibration orientations */
            float o_c[] = fuseOrientations(FILTER_CONSTANT, o_g_c, o_a_c);
            
            /* replace calculated heading with the more accurate one from the magnetometer if enabled */
            if (USE_MAGNETOMETER)
            {
              o_a_c[2] = getHeading_m(o_c[0], o_c[1]);  // (roll, pitch)
              o_c = fuseOrientations(FILTER_CONSTANT, o_g_c, o_a_c);
            }
                        
            /* set global offset vectors */
            calibrations.set(0, a_c);  // accelerations
            calibrations.set(1, o_c);  // orientation
            
            calibrationFlag = false;
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
          
          /* replace calculated heading with the more accurate one from the magnetometer if enabled */
          if (USE_MAGNETOMETER)
          {
            o_a[2] = getHeading_m(o[0], o[1]);  // (roll, pitch)
            o = fuseOrientations(FILTER_CONSTANT, o_g, o_a);
          }
          
          if (SHOW_FILTERED_ORIENTATION) println("roll = " + o[0]*180/PI + " pitch = " + o[1]*180/PI + " heading = " + o[2]*180/PI);
          
//          /* remove orientation offsets */
//          if (REMOVE_ORIENTATION_OFFSET)  // ATTENTION: only allowed when calibrated on a plane ground!
//          {
//            o[0] -= calibrations.get(1)[0];
//            o[1] -= calibrations.get(1)[1];
//            o[2] -= calibrations.get(1)[2];
//            
//            println("orientation-offset = " + calibrations.get(1)[0]*180/PI + ", " + calibrations.get(1)[1]*180/PI + ", " + calibrations.get(1)[2]*180/PI);
//          }  
          
          /* update global orientation vector */
          orientationVector.set(0, o);
          
          /* remove g offset */
          if (REMOVE_G_VECTOR)
          {
            float g[] = getGravityVector(o[0], o[1]);  // (roll, pitch)
            a[0] -= g[0];
            a[1] -= g[1];
            a[2] -= g[2];
            
            println("g-vector = " + g[0] + ", " + g[1] + ", " + g[2]);
          }
          
          float a[] = {ax, ay, az};          
          /* rotate acceleration vector to fit them to the room coordinate system */
          if (ROTATE) a = rotateArrayVector(a, o[0], o[1], o[2]);  // (a, roll, pitch, heading)


          /* update global acceleration vector */
          accelerationVector.set(0, a);
          
          /* remove orientation offsets */
          if (REMOVE_ORIENTATION_OFFSET)  // ATTENTION: only allowed when calibrated on a plane ground!
          {
            o[0] -= calibrations.get(1)[0];
            o[1] -= calibrations.get(1)[1];
            o[2] -= calibrations.get(1)[2];
            orientationVector.set(0, o);
            
            println("orientation-offset = " + calibrations.get(1)[0]*180/PI + ", " + calibrations.get(1)[1]*180/PI + ", " + calibrations.get(1)[2]*180/PI);
          }          
          
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
  float heading_a = getHeading_a();
  //println("roll_a = " + roll_a*180/PI + " pitch_a = " + pitch_a*180/PI + " heading_a = " + heading_a*180/PI);
  
  float o_a[] = {roll_a, pitch_a, heading_a};
  return o_a;
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* fuse orientations: complementary filter */
float[] fuseOrientations(float fc, float g[], float a[])  // the bigger fc the better the result but slower the system!
{
  /* calculate fc on base of current gyro magnitude if enabled */
  if (CHANGING_FILTER_CONSTANT)
  {
    float w_magnitude = sqrt(sq(data.get(0)[7]) + sq(data.get(0)[8]) + sq(data.get(0)[9]));
    fc = constrain(mapFloat(abs(w_magnitude), 0, 300, 0, 1), 0, 1);  // map and limit gyro magnitude to 0... 1
  }

  float roll    = fc * g[0] + (1 - fc) * a[0];
  float pitch   = fc * g[1] + (1 - fc) * a[1];
  float heading = fc * g[2] + (1 - fc) * a[2];
  //println("roll = " + roll*180/PI + " pitch = " + pitch*180/PI + " heading = " + heading*180/PI);
  
  /* update orientation vector */
  float o[] = {roll, pitch, heading};
  return o;
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/ 
/* get tilt compensated heading (xy-axis rotation) out of the magnetometer values */
float getHeading_m(float roll, float pitch)
{
  float Bx = data.get(0)[4];
  float By = data.get(0)[5];
  float Bz = data.get(0)[6];
  
  /* normalize magnetometer values */
  float B_amount = sqrt(Bx*Bx + By*By + Bz*Bz);
  Bx /= B_amount;
  By /= B_amount;
  Bz /= B_amount;
  
  /* magnetic heading */
  float headX = Bx*cos(pitch) + By*sin(roll)*sin(pitch) + Bz*cos(roll)*sin(pitch);  // tilt compensated magnetic field X component
  float headY = By*cos(roll) - Bz*sin(roll);                                     // tilt compensated magnetic field Y component
  float heading = atan2(-headY, headX);
  if (heading < 0) heading += 2*PI;

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

/* EXAMPLE:
//normalize the magnetic vector
norm= sqrt( sq(xMagnetMap) + sq(yMagnetMap) + sq(zMagnetMap));
xMagnetMap /=norm;
yMagnetMap /=norm;
zMagnetMap /=norm;
 
//compare "Applications of Magnetic Sensors for Low Cost Compass Systems" by Michael J. Caruso
//for the compensated Yaw equations...
//http://www.ssec.honeywell.com/magnetic/datasheets/lowcost.pdf
yawRaw=atan2( (-yMagnetMap*cos(Roll) + zMagnetMap*sin(Roll) ) , (xMagnetMap*cos(Pitch) + yMagnetMap*sin(Pitch)*sin(Roll)+ zMagnetMap*sin(Pitch)*cos(Roll)) ) *180/PI;
YawU=atan2(-yMagnetMap, xMagnetMap) *180/PI;


//apply Low Pass to Yaw
//Digital Low Pass - compare: (for accelerometer)
 //http://en.wikipedia.org/wiki/Low-pass_filter
 Yaw= yawFilteredOld + alphaYaw * (yawRaw - yawFilteredOld);
 yawFilteredOld=Yaw;
 */

/*------------------------------------------------------------------*/ 
/* get heading (xy-axis rotation) out of the acceleration values (alternative worse variant) */
float getHeading_a()
{ 
  float ax = data.get(0)[1];
  float ay = data.get(0)[2];
  float az = data.get(0)[3];
  
  float heading_a = atan2(ay, ax);
  if (heading_a < 0) heading_a += 2*PI;
  
  return heading_a;  // in rad
}
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/ 
float[] getGravityVector(float roll, float pitch)
{
  float gravity = sqrt(sq(calibrations.get(0)[0]) + sq(calibrations.get(0)[1]) + sq(calibrations.get(0)[2]));
  float gx = gravity * sin(pitch);
  float gy = gravity * cos(pitch) * sin(roll);
  float gz;
  = -gravity * cos(pitch) * cos(roll);
  
  println("g = " + gravity);
  
  float g[] = {gx, gy, gz};
  return g;
}
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/ 
int vCnt = 0;  // global counter

/* integrate acceleration vector to get the velocity vector */
float[] getVelocityVector(float dt)
{
  // v_new = v_old + da = v_old + a * dt
  float ax = accelerationVector.get(0)[0];
  float ay = accelerationVector.get(0)[1];
  float az = accelerationVector.get(0)[2];
  float vx = velocityVector.get(0)[0];
  float vy = velocityVector.get(0)[1];
  float vz = velocityVector.get(0)[2];
  
  /* take a threshold into account before integrating */
  if (abs(ax) > ACCEL_THRESHOLD)
  {
    vx += ax * dt;
    vCnt = 0;
  }
  else vCnt++;
  if (abs(ay) > ACCEL_THRESHOLD)
  {
    vy += ay * dt;
    vCnt = 0;
  }
  else vCnt++;
  if (abs(az) > ACCEL_THRESHOLD)
  {
    vz += az * dt;
    vCnt = 0;
  }
  else vCnt++;
  
  /* reset velocity if it's constant for some time */
  if (vCnt >= 10)  //blup
  {
    vx = 0;
    vy = 0;
    vz = 0;
  }
  
  float v[] = {vx, vy, vz}; 
  return v;
}
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/ 
/* integrate velocity vector to get the trail vector */
float[] getTrailVector(float dt)
{
  // s_new = s_old + dv = s_old + v * dt
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
  text("-z", 0, tl, 0);
  text("+z", 0, -l, 0);
  
  stroke(0, 0, 255);  //blue
  line(0, 0, -l, 0, 0, l);
  text("+y", 0, 0, tl);
  text("-y", 0, 0, -l);
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
void drawFloatingObject()
{
  float roll    = orientationVector.get(0)[0] + PI/2;
  float pitch   = orientationVector.get(0)[1];
  float heading = orientationVector.get(0)[2];
    
  rotateX(roll);
  rotateY(pitch);
  //rotateZ(heading);
  
  fill(0, 255, 100);  // set the disc fill color
  ellipse(0, 0, width/2, width/3);  // draw the disc
  
  fill(255, 255, 255);  // set the text fill color
  text("roll = " + round((roll-PI/2)*180/PI) + ", pitch = " + round(pitch*180/PI), -82, 10, 1);  // Draw some text so you can tell front from back
  //line(0, 0, 0, width/4*sin(roll + PI/2), width/4*cos(pitch), 0);
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
float[] rotateArrayVector(float vec[], float roll, float pitch, float heading)
{
  /* collect current data */
  float x = vec[0];
  float y = vec[1];
  float z = vec[2];
  float x_new1, y_new1, z_new1, x_new2, y_new2, z_new2;
            
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
