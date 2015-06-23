/* imports */
import processing.serial.*;

/* defines */
int X = 800;
int Y = 800;

int REF_H = 400;
int REF_B = 150;

int TEXT_SIZE = 50;
int LINE_OFFSET = 200;
  
int NUMBER_OF_SENSOR_DATA = 11;              // dt, ax, ay, az, Bx, By, Bz, Gxy, Gxz, Gyz, T (in this order!)
float GRAVITY = 9.81;                        // will be used if ENABLE_CALIBRATION is disabled
float MAX_GYRO_MAGNITUDE = 200;              // will be used if CHANGING_FILTER_CONSTANT is enabled
float FILTER_CONSTANT = 0.5;                 // will be used if CHANGING_FILTER_CONSTANT is disabled (the bigger the more you trust the gyro values)
float TAU_LP_FILTER = 0.1;                   // low pass filter time constant for the accelerations (tau is the length of signals the filter should act on)
float ACCEL_THRESHOLD = 0.5;                 // threshold in m/s^2 to observe before integrating the accelerations
int COUNTS_BEFORE_RESETTING_VELOCITY = 10;   // counts the acceleration needs to be withing defined threshold before the velocity will be resetted
int NUMBER_OF_DATA_TO_CALIBRATE = 200;

/* communication */
int BAUDRATE = 115200;
int SERIAL_PORT = 5;
boolean firstContact = false;
String s = "knock";  // handshacke key

/* switches */
boolean ENABLE_CALIBRATION = true;                // enable the calibration to get gravity magnitude and offset vectors
boolean FILTER_ACCELERATIONS = false;             // filters the accelerations with a low pass
boolean REMOVE_GYRO_OFFSET = true;                // removes angle velocity  -> ATTENTION: only allowed when calibrated quietly!
boolean CHANGING_FILTER_CONSTANT = false;         // recalculate filter constant on base of current gyro magnitude and MAX_GYRO_MAGNITUDE
boolean USE_MAGNETOMETER = false;                 // uses the more accurate magnetometer to calculate the heading instead of the acceleration vector
boolean REMOVE_G_VECTOR = true;                   // remove g-vector
boolean ROTATE = true;                            // rotate data to fit them to the room coordinate system
boolean ACCEL_NOISE_NEAR_ZERO_CANCELING = false;  // cut off the acceleration noise near 0 by taking an ACCEL_TRESHHOLD into account
boolean RESET_VELOCITY_IF_ACCEL_CONSTANT = true;  // resets the velocity if the acceleration is constant for some time
boolean REMOVE_ORIENTATION_OFFSET = false;         // remove orientation offsets -> ATTENTION: only allowed when calibrated on a plane ground!
//boolean EXP_ERROR_CORRECTION = true;              // remove exponential velocity errors

boolean SHOW_RAW_DATA = true;                     // shows raw data 
boolean SHOW_GYRO_OFFSET = true;                 // shows the orientation offsets from the calibration
boolean SHOW_NON_FILTERED_ORIENTATIONS = false;    // shows the non filtered orientations
boolean SHOW_FILTERED_ORIENTATION = false;         // shows the orientation before removing the orientation offset
boolean SHOW_G = false;                            // shows the g magnitude from the calibration
boolean SHOW_G_VECTOR = true;                     // shows the current g-vector
boolean SHOW_ORIENTATION_OFFSET = false;           // shows the orientation offsets from the calibration
//boolean SHOW_EXP_ERROR_CORRECTION = false;        // shows velocity vectors after the exponential error corrections
boolean SHOW_VECTORS = true;                      // shows the time-delta, acceleration-, velocity-, trail- and orientation vectors

boolean DISPLAY_3D_OBJECT = false;                // display 3D object which moves according the current orientation, otherwise display 2D diagrams

/* drawing multiplicators */
int ka = 10;
int kv = 20;
int ks = 50;
int kt = 5;

/* global variables */
int mX, mY;    // mouse coordinates
int scaleFactor;
float translateX;
float translateZ;

Serial myPort;

ArrayList<float[]> data = new ArrayList();                // global generic array list
ArrayList<float[]> calibrations = new ArrayList();        // global generic array list
ArrayList<float[]> orientationVector = new ArrayList();   // global generic array list
ArrayList<float[]> accelerationVector = new ArrayList();  // global generic array list
ArrayList<float[]> velocityVector = new ArrayList();      // global generic array list
ArrayList<float[]> trailVector = new ArrayList();         // global generic array list

int vectorCnt = 0;
int vectorCnt_old = 0;
int displayCnt = 1;

/* calibration variables */
boolean calibrationFlag = true;

/*------------------------------------------------------------------*/
/* setup */
void setup()
{
  /* setup window */
  if (DISPLAY_3D_OBJECT) size(X, Y, P3D);
  else size(X, Y);
  background(150, 150, 150);
  textSize(TEXT_SIZE);
  stroke(255, 0, 0);
  scaleFactor = 1;
  
  /* init serial port */
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[SERIAL_PORT], BAUDRATE);
  
  /* init first vectors with zeros */       
  float zeros[] = {0, 0, 0};
  orientationVector.add(0, zeros);   // init orientation vector
  accelerationVector.add(0, zeros);  // init acceleration vector
  velocityVector.add(0, zeros);      // init velocity vector
  trailVector.add(0, zeros);         // init trail vector
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* loop */
void draw()
{ 
  if (calibrationFlag)
  {
    text("Calibrating", X/2-5*TEXT_SIZE/2, Y/2);
  }
  else
  {
    if (DISPLAY_3D_OBJECT)
    {
      draw3DDiagramAxes();
      
      drawFloatingObject();
    }
    else 
    {
      draw2DDiagramAxes();
      
      drawVectors2D();
    }
       
    popMatrix();
  }
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
void mouseWheel(MouseEvent e)
{
    translateZ -= e.getCount() * 5;
    scaleFactor += e.getCount() / 100;
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
void keyPressed()
{
  float zeros[] = {0, 0, 0};
  velocityVector.set(vectorCnt-1, zeros);
  trailVector.set(vectorCnt-1, zeros);
  displayCnt = 1;
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
void serialEvent(Serial myPort)
{
  /* put the incoming data into a string */
  String stringVal = myPort.readStringUntil('\n');  //the '\n' is our end delimiter indicating the end of a complete packet
  
  /* make sure our data isn't empty before continuing */
  if (stringVal != null)
  {
    /* trim whitespace and formatting characters (like carriage return) */
    stringVal = trim(stringVal);  // necessary to realize the handshacke key
    //println(stringVal);
      
    if (!firstContact)
    {
      /* look for our "knock" string to start the handshake */
      if (stringVal.equals(s))
      {
        println("contact established");
        
        /* clear the buffer and send a request for data */
        myPort.clear();
        firstContact = true;
        myPort.write("who's there?");  // data request
        println("data request sent");
      }
    }
    else
    { 
      /* if we've already established contact, keep getting and parsing data */
      float da[] = (float(split(stringVal, ',')));  //parses the packet and places the stringValues into the data array
      
      /* check if data is a number */
      if (!Float.isNaN(da[0]))
      {
        /* add data */
        data.add(vectorCnt, da);
        if (SHOW_RAW_DATA) println(str(data.get(vectorCnt)));
        
        /* check calibration flag */
        if (calibrationFlag && ENABLE_CALIBRATION)
        {
          /* collect first few data for calibration */
          if (vectorCnt < NUMBER_OF_DATA_TO_CALIBRATE)
          {
            vectorCnt++;
            print("calibration data collected \t");
          }
          else
          {
            /* reset vector counter after collecting the calibration data */
            vectorCnt = 0;
            
            /* get mean values (calibration) */
            float dt_c = 0;
            float ax_c = 0; float ay_c = 0; float az_c = 0;
            float wx_c = 0; float wy_c = 0; float wz_c = 0;
                        
            for (int i = 0; i < NUMBER_OF_DATA_TO_CALIBRATE; i++)
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
            
            /* set calibration angle velocity vector */
            float w_c[] = {wx_c, wy_c, wz_c};
            
            /* get calibration orientation */
            float o_w_c[] = getOrientationGyro(vectorCnt, dt_c, wx_c, wy_c, wz_c);
            float o_a_c[] = getOrientationAccel(ax_c, ay_c, az_c);
            
            /* fuse calibration orientations */
            float o_c[] = fuseOrientations(vectorCnt, FILTER_CONSTANT, o_w_c, o_a_c);
            
            /* replace calculated heading with the more accurate one from the magnetometer if enabled */
            if (USE_MAGNETOMETER)
            {
              o_a_c[2] = getHeading(vectorCnt, o_c[0], o_c[1]);  // (roll, pitch)
              o_c = fuseOrientations(vectorCnt, FILTER_CONSTANT, o_w_c, o_a_c);
            }
                        
            /* set global offset vectors */
            calibrations.add(0, a_c);  // accelerations
            calibrations.add(1, w_c);  // angle velocity
            calibrations.add(2, o_c);  // orientation
            
            calibrationFlag = false;
          }
        }
        else
        {
        /* data processing: */
          /* get dt */
          float dt = data.get(vectorCnt)[0] / 1000000;  // in s
          
          /* get a */
          float ax = data.get(vectorCnt)[1];
          float ay = data.get(vectorCnt)[2];
          float az = data.get(vectorCnt)[3];
          float a[] = {ax, ay, az}; 
          
          /* filter a */
          float LP_filterConstant = TAU_LP_FILTER / (TAU_LP_FILTER + dt);
          if (FILTER_ACCELERATIONS && vectorCnt != 0) a = lowPassFilter(LP_filterConstant, accelerationVector.get(vectorCnt-1), a);
          
          /* get w */
          float wx = data.get(vectorCnt)[7];
          float wy = data.get(vectorCnt)[8];
          float wz = data.get(vectorCnt)[9];
          
          /* remove gyro offset */
          if (REMOVE_GYRO_OFFSET)  // ATTENTION: only allowed when calibrated quietly!
          {
            wx -= calibrations.get(1)[0];
            wy -= calibrations.get(1)[1];
            wz -= calibrations.get(1)[2];
            
            if (SHOW_GYRO_OFFSET) println("gyro-offset = " + calibrations.get(1)[0] + ", " + calibrations.get(1)[1] + ", " + calibrations.get(1)[2]);
          }     
          
          /* get orientations */
          float o_w[] = getOrientationGyro(vectorCnt, dt, wx, wy, wz);
          float o_a[] = getOrientationAccel(ax, ay, az);
          
          /* fuse orientations */
          float o[] = fuseOrientations(vectorCnt, FILTER_CONSTANT, o_w, o_a);
          
          /* replace calculated heading with the more accurate one from the magnetometer if enabled */
          if (USE_MAGNETOMETER)
          {
            o_a[2] = getHeading(vectorCnt, o[0], o[1]);  // (roll, pitch)
            o = fuseOrientations(vectorCnt, FILTER_CONSTANT, o_w, o_a);
          }
                    
          /* remove tilt compensated g offset */
          if (REMOVE_G_VECTOR)
          {
            float g[] = getGravityVector(o[0], o[1]);  // (roll, pitch)
            a[0] -= g[0];
            a[1] -= g[1];
            a[2] -= g[2];
            
            if (SHOW_G_VECTOR) println("g-vector = " + g[0] + ", " + g[1] + ", " + g[2]);
          }
          
          /* rotate acceleration vector to fit them to the room coordinate system */
          if (ROTATE) a = rotateArrayVector(a, o[0], o[1], o[2]);  // (a, roll, pitch, heading)
          
          /* cut off noise near 0 by taking a accelerometer threshold into account */
          if (ACCEL_NOISE_NEAR_ZERO_CANCELING)
          {
            if (abs(a[0]) < ACCEL_THRESHOLD) a[0] = 0;
            if (abs(a[1]) < ACCEL_THRESHOLD) a[1] = 0;
            if (abs(a[2]) < ACCEL_THRESHOLD) a[2] = 0;
          }
          
          /* update global acceleration vector */
          accelerationVector.add(vectorCnt, a);
          
          /* remove orientation offsets */
          if (REMOVE_ORIENTATION_OFFSET)  // ATTENTION: only allowed when calibrated on a plane ground!
          {
            o[0] -= calibrations.get(2)[0];
            o[1] -= calibrations.get(2)[1];
            o[2] -= calibrations.get(2)[2];
            
            if (SHOW_ORIENTATION_OFFSET) println("orientation-offset = " + calibrations.get(2)[0]*180/PI + ", " + calibrations.get(2)[1]*180/PI + ", " + calibrations.get(2)[2]*180/PI);
          }          
          
          /* update global orientation vector */
          orientationVector.add(vectorCnt, o);
          
          /* get velocity vecotr */
          float v[] = getVelocityVector(vectorCnt, dt);
          velocityVector.add(vectorCnt, v);  // update global velocity vector
          
          /* get trail vector */
          float s[] = getTrailVector(vectorCnt, dt); 
          trailVector.add(vectorCnt, s);  // update global trail vector
          
          /* display data */
          showVectors(vectorCnt);
                    
          /* update vector counter */
          vectorCnt++;
        }
      }
    }
  }
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
float[] lowPassFilter(float fc, float vec_old[], float vec_new[])
{
  vec_old[0] = fc * vec_new[0] + (1 - fc) * vec_old[0];
  vec_old[1] = fc * vec_new[1] + (1 - fc) * vec_old[1];
  vec_old[2] = fc * vec_new[2] + (1 - fc) * vec_old[2];
  
  return vec_old;
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* integrate gyro values to get orientations in rad */
float[] getOrientationGyro(int vectorNr, float dt, float wx, float wy, float wz)
{ 
  if (vectorNr == 0) vectorNr = 1;
  
  // orientation_new = orientation_old + w * dt
  float roll_w    = orientationVector.get(vectorNr-1)[0] + wx * dt * PI/180;  // fPhi(wx)
  float pitch_w   = orientationVector.get(vectorNr-1)[1] + wy * dt * PI/180;  // fTheta(wy)
  float heading_w = orientationVector.get(vectorNr-1)[2] + wz * dt * PI/180;  // fPsi(wz)
  if (SHOW_NON_FILTERED_ORIENTATIONS) println("roll_w = " + roll_w*180/PI + " pitch_w = " + pitch_w*180/PI + " heading_w = " + heading_w*180/PI);
  
  float o_w[] = {roll_w, pitch_w, heading_w};
  return o_w;
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* calculate orientations in rad from the acceleration vector */
float[] getOrientationAccel(float ax, float ay, float az)
{  
  float u = 0.001;  // to prevent that ax and az could be simultaneously zero and give an undefined or unstable estimate of the roll angle
  
  /* normalize acceleratons */
  float a_amount = sqrt(ax*ax + ay*ay + az*az);
  ax /= a_amount;
  ay /= a_amount;
  az /= a_amount;
  
  float roll_a    = atan2(ay, sign(az)*sqrt(az*az + u*ax*ax));  //atan2(ay, az);  // get roll (yz-axis rotation) -> -PI... PI
  float pitch_a   = atan2(ax, sqrt(ay*ay + az*az));             // get pitch (xz-axis rotation) -> -PI/2... PI/2  //blup -PI... PI would be better!
  float heading_a = atan2(ay, ax);                              // get heading (xy-axis rotation) -> -PI... PI
  
  if (SHOW_NON_FILTERED_ORIENTATIONS) println("roll_a = " + roll_a*180/PI + " pitch_a = " + pitch_a*180/PI + " heading_a = " + heading_a*180/PI);
  
  float o_a[] = {roll_a, pitch_a, heading_a};
  return o_a;
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* fuse orientations: complementary filter */
float[] fuseOrientations(int vectorNr, float fc, float o_w[], float o_a[])  // the bigger fc the better the result but slower the system!
{
  /* calculate fc on base of current gyro magnitude if enabled */
  if (CHANGING_FILTER_CONSTANT)
  {
    float w_magnitude = sqrt(sq(data.get(vectorNr)[7]) + sq(data.get(vectorNr)[8]) + sq(data.get(vectorNr)[9]));
    fc = constrain(mapFloat(w_magnitude, 0, MAX_GYRO_MAGNITUDE, 0, 1), 0, 1);  // map and limit gyro magnitude to 0... 1
  }

  float roll    = fc * o_w[0] + (1 - fc) * o_a[0];
  float pitch   = fc * o_w[1] + (1 - fc) * o_a[1];
  float heading = fc * o_w[2] + (1 - fc) * o_a[2];
  if (SHOW_FILTERED_ORIENTATION) println("roll = " + roll*180/PI + " pitch = " + pitch*180/PI + " heading = " + heading*180/PI);
  
  /* update orientation vector */
  float o[] = {roll, pitch, heading};
  return o;
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/ 
/* get tilt compensated heading (xy-axis rotation) out of the magnetometer values */
float getHeading(int vectorNr, float roll, float pitch)
{
  float Bx = data.get(vectorNr)[4];
  float By = data.get(vectorNr)[5];
  float Bz = data.get(vectorNr)[6];
  
  /* normalize magnetometer values */
  float B_amount = sqrt(Bx*Bx + By*By + Bz*Bz);
  Bx /= B_amount;
  By /= B_amount;
  Bz /= B_amount;
  
  /* magnetic heading */
  float headX = Bx*cos(pitch) + By*sin(roll)*sin(pitch) + Bz*cos(roll)*sin(pitch);  // tilt compensated magnetic field X component
  float headY = By*cos(roll) - Bz*sin(roll);                                        // tilt compensated magnetic field Y component
  float heading = atan2(-headY, headX);
  if (heading < 0) heading += 2*PI;  //blup

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
float[] getGravityVector(float roll, float pitch)
{
  float gravity = GRAVITY;
  if (ENABLE_CALIBRATION) gravity = sqrt(sq(calibrations.get(0)[0]) + sq(calibrations.get(0)[1]) + sq(calibrations.get(0)[2]));
  
  float gx = gravity * sin(pitch);
  float gy = gravity * cos(pitch) * sin(roll);
  float gz = gravity * cos(pitch) * cos(roll);
  
  if (SHOW_G) println("g = " + gravity);
  
  float g[] = {gx, gy, gz};
  return g;
}
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/ 
int vXCnt = 0;  // global counter
int vYCnt = 0;  // global counter
int vZCnt = 0;  // global counter
/* integrate acceleration vector to get the velocity vector */
float[] getVelocityVector(int vectorNr, float dt)
{
  // v_new = v_old + da = v_old + a * dt
  float ax = accelerationVector.get(vectorNr)[0];
  float ay = accelerationVector.get(vectorNr)[1];
  float az = accelerationVector.get(vectorNr)[2];
  
  if (vectorNr == 0) vectorNr = 1;
  float vx = velocityVector.get(vectorNr-1)[0];
  float vy = velocityVector.get(vectorNr-1)[1];
  float vz = velocityVector.get(vectorNr-1)[2];
  
  if (RESET_VELOCITY_IF_ACCEL_CONSTANT)
  {
    /* take a threshold into account before integrating */
    if (abs(ax) > ACCEL_THRESHOLD)
    {
      vx += ax * dt;
      vXCnt = 0;
    }
    else vXCnt++;
    if (abs(ay) > ACCEL_THRESHOLD)
    {
      vy += ay * dt;
      vYCnt = 0;
    }
    else vYCnt++;
    if (abs(az) > ACCEL_THRESHOLD)
    {
      vz += az * dt;
      vZCnt = 0;
    }
    else vZCnt++;
    
    /* reset velocity if acceleration is constant for some time */
    if (vXCnt >= COUNTS_BEFORE_RESETTING_VELOCITY) vx = 0;
    if (vYCnt >= COUNTS_BEFORE_RESETTING_VELOCITY) vy = 0;
    if (vZCnt >= COUNTS_BEFORE_RESETTING_VELOCITY) vz = 0;
  }
  else
  {
    vx += ax * dt;
    vy += ay * dt;
    vz += az * dt;
  }
  
  float v[] = {vx, vy, vz}; 
  return v;
}
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/ 
/* integrate velocity vector to get the trail vector */
float[] getTrailVector(int vectorNr, float dt)
{
  if (vectorNr == 0) vectorNr = 1;
  
  // s_new = s_old + dv = s_old + v * dt
  float sx = trailVector.get(vectorNr-1)[0] + velocityVector.get(vectorNr)[0] * dt;
  float sy = trailVector.get(vectorNr-1)[1] + velocityVector.get(vectorNr)[1] * dt;
  float sz = trailVector.get(vectorNr-1)[2] + velocityVector.get(vectorNr)[2] * dt;
  
  float s[] = {sx, sy, sz}; 
  return s;
}
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/   
void showVectors(int vectorNr)
{
  if (SHOW_VECTORS)
  {
    println("show vectors:"); print("dt"); print("\t"); print("ax, ay, az");
    print("\t\t\t\t\t"); print("vx, vy, vz"); print("\t\t\t\t"); println("sx, sy, sz");
    
    print(str(data.get(vectorNr)[0]));
    print("\t");
    print(str(accelerationVector.get(vectorNr)));
    print("\t\t");
    print(str(velocityVector.get(vectorNr)));
    print("\t\t");
    println(str(trailVector.get(vectorNr)));
    println("roll = " + orientationVector.get(vectorNr)[0]*180/PI + " pitch = " + orientationVector.get(vectorNr)[1]*180/PI +
            " heading = " + orientationVector.get(vectorNr)[2]*180/PI);
    println("");
  }
}
/*------------------------------------------------------------------*/ 


// draw functions
/*------------------------------------------------------------------*/  
void draw2DDiagramAxes()
{
  background(0);
  
  pushMatrix();
  scale(scaleFactor);
  
  if(mousePressed)
  {
    mX = mouseX;
    mY = mouseY;
  }
  
  translate(0, LINE_OFFSET);
  
  draw2DAxes(X/10*9, 30);
}
/*------------------------------------------------------------------*/  

/*------------------------------------------------------------------*/
void draw2DAxes(int l, int to)
{
  textSize(TEXT_SIZE/2);
  
  fill(255, 255, 255);
  stroke(255, 0, 0);  // red
  line(0, 0, l, 0);
  text("+tx", l, 0);
  
  line(0, LINE_OFFSET, l, LINE_OFFSET);
  text("+ty", l, LINE_OFFSET);
  
  line(0, 2*LINE_OFFSET, l, 2*LINE_OFFSET);
  text("+tz", l, 2*LINE_OFFSET);
  
  stroke(0, 255, 0);  // green
  line(0, -l, 0, l);
  text("+var", 0, l-to);
  text("-var", 0, -LINE_OFFSET+to);
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
  
  draw3DAxes(X, 30);
}
/*------------------------------------------------------------------*/  

/*------------------------------------------------------------------*/
void draw3DAxes(int l, int textOffset)
{
  textSize(TEXT_SIZE/2);
  
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

/*------------------------------------------------------------------*/
ArrayList<float[]> ma = new ArrayList();
ArrayList<float[]> mv = new ArrayList();
ArrayList<float[]> ms = new ArrayList();
ArrayList<float[]> mt = new ArrayList();
void drawVectors2D()
{
  float zeros_m[] = {0, 0, 0};
  if (displayCnt == 1)
  {
    ma.add(0, zeros_m);
    mv.add(0, zeros_m);
    ms.add(0, zeros_m);
    mt.add(0, zeros_m);
  }
  
  if (vectorCnt_old != vectorCnt)
  { 
    //float ka = 10;
    float ax = accelerationVector.get(vectorCnt_old)[0];
    float ay = accelerationVector.get(vectorCnt_old)[1];
    float az = accelerationVector.get(vectorCnt_old)[2];
    float a_old = ma.get(displayCnt-1)[0];
    float a_new[] = {ax*ka, ay*ka, az*ka};
    ma.add(displayCnt, a_new);    
    
    //float kv = 10;
    float vx = velocityVector.get(vectorCnt_old)[0];
    float vy = velocityVector.get(vectorCnt_old)[1];
    float vz = velocityVector.get(vectorCnt_old)[2];
    float v_old = mv.get(displayCnt-1)[0];
    float v_new[] = {vx*kv, vy*kv, vz*kv};
    mv.add(displayCnt, v_new);
    
    //float ks = 50;
    float sx = trailVector.get(vectorCnt_old)[0];
    float sy = trailVector.get(vectorCnt_old)[1];
    float sz = trailVector.get(vectorCnt_old)[2];
    float s_old = ms.get(displayCnt-1)[0];
    float s_new[] = {vx*ks, vy*ks, vz*ks};
    ms.add(displayCnt, s_new);
    
    //float kt = 5;
    float dt = data.get(vectorCnt_old)[0] / 1000000 * 10;
    float t_old = mt.get(displayCnt-1)[0];
    float t_new[] = {t_old + dt*kt};
    mt.add(displayCnt, t_new);

    displayCnt++;
    vectorCnt_old = vectorCnt;
  }
  
  for (int n = 1; n < displayCnt; n++)
  {
    stroke(50, 50, 255);
    line(mt.get(n-1)[0], ma.get(n-1)[0], mt.get(n)[0], ma.get(n)[0]);
    line(mt.get(n-1)[0], ma.get(n-1)[1]+LINE_OFFSET, mt.get(n)[0], ma.get(n)[1]+LINE_OFFSET);
    line(mt.get(n-1)[0], ma.get(n-1)[2]+2*LINE_OFFSET, mt.get(n)[0], ma.get(n)[2]+2*LINE_OFFSET);
      
    stroke(50, 250, 55);
    line(mt.get(n-1)[0], mv.get(n-1)[0], mt.get(n)[0], mv.get(n)[0]);
    line(mt.get(n-1)[0], mv.get(n-1)[1]+LINE_OFFSET, mt.get(n)[0], mv.get(n)[1]+LINE_OFFSET);
    line(mt.get(n-1)[0], mv.get(n-1)[2]+2*LINE_OFFSET, mt.get(n)[0], mv.get(n)[2]+2*LINE_OFFSET);
    
    stroke(250, 50, 55);
    line(mt.get(n-1)[0], ms.get(n-1)[0], mt.get(n)[0], ms.get(n)[0]);
    line(mt.get(n-1)[0], ms.get(n-1)[1]+LINE_OFFSET, mt.get(n)[0], ms.get(n)[1]+LINE_OFFSET);
    line(mt.get(n-1)[0], ms.get(n-1)[2]+2*LINE_OFFSET, mt.get(n)[0], ms.get(n)[2]+2*LINE_OFFSET);
    
        stroke(50, 250, 55);
    line(mt.get(n-1)[0], mv.get(n-1)[0], mt.get(n)[0], mv.get(n)[0]);
    line(mt.get(n-1)[0], mv.get(n-1)[1]+LINE_OFFSET, mt.get(n)[0], mv.get(n)[1]+LINE_OFFSET);
    line(mt.get(n-1)[0], mv.get(n-1)[2]+2*LINE_OFFSET, mt.get(n)[0], mv.get(n)[2]+2*LINE_OFFSET);
  }
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
void drawFloatingObject()
{
  int cnt = vectorCnt;
  if (cnt == 0) cnt = 1;
  float roll    = orientationVector.get(cnt-1)[0] + PI/2;
  float pitch   = orientationVector.get(cnt-1)[1];
  float heading = orientationVector.get(cnt-1)[2];
    
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

// auxiliary functions:
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
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
float sign(float value)
{
  if (value > 0) return 1;
  else return -1;
}
/*------------------------------------------------------------------*/
