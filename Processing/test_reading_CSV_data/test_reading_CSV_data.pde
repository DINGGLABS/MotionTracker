/* imports */
import java.util.Date;

/* defines */
int X = 600;
int Y = 600;

int REF_H = 400;
int REF_B = 150;

String FILENAME = "data/achti.csv";

int NUMBER_OF_SENSOR_DATA = 8;  // t, ax, ay, az, h, Gxy, Gxz, Gyz (in this order!)
int NUMBER_OF_DATA_USED_TO_CALIBRATE = 150;  // that means 5sec calibrating!

boolean ROTATE = true;
boolean G_ROTATION = true;
boolean A_OFFSET = true;

boolean G_OFFSET = false;
boolean TRAPEZ = false;    // alternative integration
boolean EXP_ERROR_CORRECTION = false;

int ma = 10;
int mv = 10;
int ms = 10;

/* global variables */
int mX, mY;    // mouse coordinates
int scaleFactor;
float translateZ;

int numberOfRows;
ArrayList<float[]> data = new ArrayList();                // global generic array list
ArrayList<float[]> accelerationVectors = new ArrayList();  // global generic array list
ArrayList<float[]> velocityVectors = new ArrayList();      // global generic array list
ArrayList<float[]> trailVectors = new ArrayList();         // global generic array list

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
  
/*------------------------------------------------------------------*/
  /* get data */
  println("get data");
  String dataRows[] = loadStrings(FILENAME);
  
  numberOfRows = dataRows.length;
  //float data[][] = new float[numberOfRows][NUMBER_OF_SENSOR_DATA];
  //float trailVectors[][] = new float[numberOfRows][3];
    
  /* print total number of rows */
  println(dataRows.length + " total data rows"); 
  
  /* convert string data to float arrays */
  for (int i = 0; i < numberOfRows; i++)
  {
    data.add(float(split(trim(dataRows[i]), ',')));  // extract and clean (trim and split) data rows
    //println(str(data.get(i)));
  }
/*------------------------------------------------------------------*/ 
  
/*------------------------------------------------------------------*/ 
  /* convert unix timestamp (last 6 digits) to seconds */
  println("convert unix timestamp");
  for (int i = 0; i < numberOfRows; i++)
  {
    data.get(i)[0] /= 1000;
    //println(str(data.get(i)));
  }
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/ 
  /* rotate data vectors to fit them to the room coordinate system */
  if (ROTATE)
  {
    println("rotate data vectors");
    for (int i = 0; i < numberOfRows; i++)
    {
      /* collect current data */
      float a[] = {data.get(i)[1], data.get(i)[2], data.get(i)[3]};
      float heading = data.get(i)[5];  // z-axis rotation angle
      float pitch = data.get(i)[6];    // y-axis rotation angle
      float roll = data.get(i)[7];     // x-axis rotation angle
              
      //println(str(data.get(i)));
      //println(sqrt(data.get(i)[1]*data.get(i)[1] + data.get(i)[2]*data.get(i)[2] + data.get(i)[3]*data.get(i)[3]));
      
      /* rotate */
      a = rotateArrayVector(a, roll, pitch, heading);
      
      /* save rotated data */
      data.get(i)[1] = a[0];
      data.get(i)[2] = a[1];
      data.get(i)[3] = a[2];
      
      //println(str(data.get(i)));
      //println(sqrt(data.get(i)[1]*data.get(i)[1] + data.get(i)[2]*data.get(i)[2] + data.get(i)[3]*data.get(i)[3]));
    }
  }
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/ 
  /* CALIBRATE_DATA data */
  println("calibrate data");
  
  float ax_c = 0;
  float ay_c = 0;
  float az_c = 0;
  float Gx_c = 0;
  float Gy_c = 0;
  float Gz_c = 0;
  
  /* get mean values (g-vector) */
  for (int i = 0; i < NUMBER_OF_DATA_USED_TO_CALIBRATE; i++)
  {
    ax_c += data.get(i)[1]; ay_c += data.get(i)[2]; az_c += data.get(i)[3];
    Gx_c += data.get(i)[5]; Gy_c += data.get(i)[6]; Gz_c += data.get(i)[7];
  }
  ax_c /= NUMBER_OF_DATA_USED_TO_CALIBRATE; ay_c /= NUMBER_OF_DATA_USED_TO_CALIBRATE; az_c /= NUMBER_OF_DATA_USED_TO_CALIBRATE;
  Gx_c /= NUMBER_OF_DATA_USED_TO_CALIBRATE; Gy_c /= NUMBER_OF_DATA_USED_TO_CALIBRATE; Gz_c /= NUMBER_OF_DATA_USED_TO_CALIBRATE;
  
  print("g-vector: ");
  println(ax_c + ", " + ay_c + ", " + az_c);
  print("g-vector magnitude: ");
  println(sqrt(ax_c*ax_c + ay_c*ay_c + az_c*az_c));  // should be more or less 9.81 !
  
  /* get g-vector angles */
//    float g_roll    = acos(ax_c/(sqrt(ay_c*ay_c + az_c*az_c))) * 180/PI;  // x-axis rotation angle
//    float g_pitch   = acos(ay_c/(sqrt(ax_c*ax_c + az_c*az_c))) * 180/PI;  // y-axis rotation angle
//    float g_heading = acos(az_c/(sqrt(ax_c*ax_c + ay_c*ay_c))) * 180/PI;  // z-axis rotation angle

  // phi = acos((a o b) / (|a| * |b|)) ; a = (gx, gy, gz) ; b1 = (0, gy, gz) ; b2 = (gx, 0, gz) ; b3 = (gx, gy, 0)
  float g_roll    = acos((ay_c*ay_c + az_c*az_c)/(sqrt((ay_c*ay_c + az_c*az_c) * (ax_c*ax_c + ay_c*ay_c + az_c*az_c)))) * 180/PI;  // x-axis rotation angle
  float g_pitch   = acos((ax_c*ax_c + az_c*az_c)/(sqrt((ax_c*ax_c + az_c*az_c) * (ax_c*ax_c + ay_c*ay_c + az_c*az_c)))) * 180/PI;  // y-axis rotation angle
  float g_heading = acos((ax_c*ax_c + ay_c*ay_c)/(sqrt((ax_c*ax_c + ay_c*ay_c) * (ax_c*ax_c + ay_c*ay_c + az_c*az_c)))) * 180/PI;  // z-axis rotation angle
  //println((sqrt((ax_c*ax_c + ay_c*ay_c) * (ax_c*ax_c + ay_c*ay_c + az_c*az_c))));
  if(g_roll != g_roll) { g_roll = 0; }
  if(g_pitch != g_pitch) { g_pitch = 0; }
  if(g_heading != g_heading) { g_heading = 0; }
  println("g-roll: " + g_roll + ", " + "g-pitch: " + g_pitch + ", " + "g-heading: " + g_heading);
    
  for (int i = 0; i < numberOfRows; i++)
  {
    /* recalculate data values */
    if (A_OFFSET)
    {
      data.get(i)[1] -= ax_c;  // remove offsets (g-shares)
      data.get(i)[2] -= ay_c;
      data.get(i)[3] -= az_c;
    }
    
//      println("after offsets");
//      print(str(data.get(i)));
//      print("\t");
    
//      if (i < 10) data.get(i)[4] = data.get(10)[4];                            // set first 10 values equal the 10th

    if (G_OFFSET)
    {
      data.get(i)[5] -= Gx_c;  // remove offsets
      data.get(i)[6] -= Gy_c;
      data.get(i)[7] -= Gz_c;
    }
          
    //println(str(data.get(i)));
    //println(sqrt(data.get(i)[1]*data.get(i)[1] + data.get(i)[2]*data.get(i)[2] + data.get(i)[3]*data.get(i)[3]));
    
    /* rotate by the g-error */
    float a[] = {data.get(i)[1], data.get(i)[2], data.get(i)[3]};
    a = rotateArrayVector(a, g_roll, g_pitch, g_heading);
    
    /* save rotated data */
    if (G_ROTATION)
    {
      data.get(i)[1] = a[0];
      data.get(i)[2] = a[1];
      data.get(i)[3] = a[2];
    }
    
//      println(str(data.get(i)));
    
    //println(str(data.get(i)));
    //println(sqrt(data.get(i)[1]*data.get(i)[1] + data.get(i)[2]*data.get(i)[2] + data.get(i)[3]*data.get(i)[3]));
  }
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/ 
  /* set acceleration vectors */
  println("set acceleration vectors");
  for (int i = 0; i < numberOfRows; i++)
  {  
    float a[] = {data.get(i)[1], data.get(i)[2], data.get(i)[3]};
    accelerationVectors.add(i, a);
  }
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/   
  /* calculate velocity vectors */
  println("calculate velocity vectors");
  float zeros[] = {0, 0, 0};
  velocityVectors.add(0, zeros);  // init first position with zeros
  
  for (int i = 1; i < numberOfRows; i++)
  {
    /* get delta t */
    float dt = data.get(i)[0] - data.get(i-1)[0];
    //println(dt);
   
   float[] delta_winkel = { data.get(i)[7] - data.get(i-1)[7], data.get(i)[6] - data.get(i-1)[6], data.get(i)[5] - data.get(i-1)[5] };  //blup
   float[] v_alt_ungedreht = {velocityVectors.get(i-1)[0], velocityVectors.get(i-1)[1], velocityVectors.get(i-1)[2]};  //blup
   float[] v_alt_gedreht = v_alt_ungedreht;//rotateArrayVector(v_alt_ungedreht, delta_winkel[0], delta_winkel[1], delta_winkel[2]);
   
//   print(str(delta_winkel));
//   print("\t");
//   print(str(v_alt_ungedreht));
//   print("\t");
//   print(str(v_alt_gedreht));
//   print("\t");
   
    float vx_neu, vy_neu, vz_neu;
    if (TRAPEZ)
    {
      /* trapezoidal rule:
      /* v_neu = v_alt + (a2 + a1)/2 * dt */
      vx_neu = velocityVectors.get(i-1)[0] + (accelerationVectors.get(i)[0] + accelerationVectors.get(i-1)[0])/2 * dt;
      vy_neu = velocityVectors.get(i-1)[1] + (accelerationVectors.get(i)[1] + accelerationVectors.get(i-1)[1])/2 * dt;
      vz_neu = velocityVectors.get(i-1)[2] + (accelerationVectors.get(i)[2] + accelerationVectors.get(i-1)[2])/2 * dt;
    }
    else
    {
      /* v_neu = v_alt + dv = v_alt + (a * dt) */
      vx_neu = v_alt_gedreht[0] + accelerationVectors.get(i)[0] * dt;
      vy_neu = v_alt_gedreht[1] + accelerationVectors.get(i)[1] * dt;
      vz_neu = v_alt_gedreht[2] + accelerationVectors.get(i)[2] * dt;
    }
    
    float v[] = {vx_neu, vy_neu, vz_neu}; 
    velocityVectors.add(i, v);
    
//    print(i + ":");
//    print("\t");
//    println(str(velocityVectors.get(i)));
  }
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/ 
  /* velocity exponential error correction */
  if (EXP_ERROR_CORRECTION)
  {
    println("velocity exponential error correction");
    float vx_start = velocityVectors.get(2)[0];
    float vy_start = velocityVectors.get(2)[1];
    float vz_start = velocityVectors.get(2)[2];
    float vx_end = velocityVectors.get(numberOfRows-1)[0];
    float vy_end = velocityVectors.get(numberOfRows-1)[1];
    float vz_end = velocityVectors.get(numberOfRows-1)[2];
    
    double expValue_x = root(numberOfRows, abs(vx_end)/abs(vx_start));  // e_v = sqrt(x, y/y0)
    double expValue_y = root(numberOfRows, abs(vy_end)/abs(vy_start));
    double expValue_z = root(numberOfRows, abs(vz_end)/abs(vz_start));
    
    if (expValue_x < 1) expValue_x = 1; if (expValue_y < 1) expValue_y = 1; if (expValue_z < 1) expValue_z = 1;
    
    println("vx_start = " + vx_start);
    println("vx_end = " + vx_end);
    println("expValue_x = " + expValue_x);
    println("");
    
    for (int i = 1; i < numberOfRows; i++)
    {
      println("i = " + i);
      println("vx_alt = " + velocityVectors.get(i)[0]);
      println("minus_x = " + (vx_start * pow((float)expValue_x, (float)i)));
      
      float vx_neu, vy_neu, vz_neu;
  
      /* y = y - y0 * e_v^x */
      float minus_x = vx_start * pow((float)expValue_x, (float)i);
      if (minus_x > velocityVectors.get(i)[0]) vx_neu = 0;
      else vx_neu = velocityVectors.get(i)[0] - minus_x;  
      
      float minus_y = vy_start * pow((float)expValue_y, (float)i);
      if (abs(minus_y) > abs(velocityVectors.get(i)[1])) vy_neu = 0;
      else vy_neu = velocityVectors.get(i)[1] - minus_y;  // y = y - y0 * e_v^x
      
      float minus_z = vz_start * pow((float)expValue_z, (float)i);
      if (abs(minus_z) > abs(velocityVectors.get(i)[2])) vz_neu = 0;
      else vz_neu = velocityVectors.get(i)[2] - minus_z;  // y = y - y0 * e_v^x
          
      float v[] = {vx_neu, vy_neu, vz_neu}; 
      velocityVectors.set(i, v);
      
      println("vx_neu = " + velocityVectors.get(i)[0]);
      println("");
      
  //    print(i + ":");
  //    print("\t");
  //    println(str(velocityVectors.get(i)));
    }
    
    println("new vx_end = " + velocityVectors.get(numberOfRows-1)[0]);
  }
/*------------------------------------------------------------------*/ 
/*

*/
/*------------------------------------------------------------------*/   
  /* calculate trail vectors */
  println("calculate trail vectors");
  trailVectors.add(0, zeros);     // init first position with zeros 
  
  for (int i = 1; i < numberOfRows; i++)
  {
    /* get delta t */
    float dt = data.get(i)[0] - data.get(i-1)[0];
    //println(dt);
          
    float sx_neu, sy_neu, sz_neu;
    if (TRAPEZ)
    {
      /* s_neu = s_alt + ds = s_alt + (v_neu * dt) */
      sx_neu = trailVectors.get(i-1)[0] + velocityVectors.get(i)[0] * dt;
      sy_neu = trailVectors.get(i-1)[1] + velocityVectors.get(i)[1] * dt;
      sz_neu = trailVectors.get(i-1)[2] + velocityVectors.get(i)[2] * dt;
    }
    else
    {
      /* trapezoidal rule: s_neu = s_alt + (v2 + v1)/2 * dt */
      sx_neu = trailVectors.get(i-1)[0] + (velocityVectors.get(i)[0] + velocityVectors.get(i-1)[0])/2 * dt;
      sy_neu = trailVectors.get(i-1)[1] + (velocityVectors.get(i)[1] + velocityVectors.get(i-1)[1])/2 * dt;
      sz_neu = trailVectors.get(i-1)[2] + (velocityVectors.get(i)[2] + velocityVectors.get(i-1)[2])/2 * dt;
    }
    
    float s[] = {sx_neu, sy_neu, sz_neu};
    trailVectors.add(i, s);
    
//    print(i + ":");
//    print("\t");
//    println(str(trailVectors.get(i)));
  }
/*------------------------------------------------------------------*/ 

///*------------------------------------------------------------------*/   
//  /* show vectors */
//  println("show vectors:"); print("id"); print("\t"); print("time"); print("\t"); print("acceleration vectors");
//  print("\t\t\t\t"); print("velocity vectors"); print("\t\t\t\t"); println("trail vectors");
//  
//  for (int i = 0; i < numberOfRows; i++)
//  { 
//    print(i + ":");
//    print("\t");
//    print(str(data.get(i)[0]));
//    print("\t");
//    print(str(accelerationVectors.get(i)));
//    print("\t\t");
//    print(str(velocityVectors.get(i)));
//    print("\t\t");
//    println(str(trailVectors.get(i)));
//  }
///*------------------------------------------------------------------*/ 
}



/* loop */
void draw()
{
///*------------------------------------------------------------------*/  
//  /* draw 2D-diagram axes */
//  background(0);
//  translate(width/4, height/4, -height/2);
//  
//  pushMatrix();
//  translate(0, 0, translateZ);
//  scale(scaleFactor);
//  
//  if(mousePressed)
//  {
//    mX = mouseY;
//    mY = mouseX;
//  }
//  
//  rotateX(map(mX, 0, height, -PI, PI));
//  rotateY(map(mY, 0, height, -PI, PI));
//  
//  draw2DAxes(X, 30);
///*------------------------------------------------------------------*/  
//  
///*------------------------------------------------------------------*/  
//  /* draw 2D-diagram */
//  stroke(255, 0, 255);  // violett
//  float m = 10;  // multiplicator
//  
//
//  float t_alt = 0;
//  float var_alt = 0;
//  for (int i = 1; i < numberOfRows; i++)
//  {
//    float t_neu = t_alt + (data.get(i)[0] - data.get(i-1)[0]) * m;
//    
//    /* magnitude of acceleration vector */
//    float x = accelerationVectors.get(i)[0] * m;
//    float y = accelerationVectors.get(i)[1] * m;
//    float z = accelerationVectors.get(i)[2] * m;
//
////    /* magnitude of velocity vector */
////    float x = velocityVectors.get(i)[0] * m;
////    float y = velocityVectors.get(i)[1] * m;
////    float z = velocityVectors.get(i)[2] * m;
//
////    /* magnitude of trail vector */
////    float x = trailVectors.get(i)[0] * m;
////    float y = trailVectors.get(i)[1] * m;
////    float z = trailVectors.get(i)[2] * m;
//    
//    float var_neu = sqrt(x*x + y*y + z*z);
//    
//    /* draw lines */
//    line(t_alt, var_alt, t_neu, var_neu);
//    
//    t_alt = t_neu;
//    var_alt = var_neu;
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
///*------------------------------------------------------------------*/  
  
  
  
  
  
/*------------------------------------------------------------------*/  
  /* draw 3D-diagram axes */
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
/*------------------------------------------------------------------*/  

///*------------------------------------------------------------------*/  
//  /* draw reference trail */
//  stroke(255, 255, 255);
//  line(0,0,0,REF_H);          // x = pos, z = pos
//  line(0,REF_H,REF_B,REF_H);
//  line(REF_B,REF_H,REF_B,0);
//  line(REF_B,0,0,0);
//  
////  line(0,0,0,REF_H);          // x = neg, z = pos
////  line(0,REF_H,-REF_B,REF_H);
////  line(-REF_B,REF_H,-REF_B,0);
////  line(-REF_B,0,0,0);
//
////  line(0,0,0,-REF_H);          // x = pos, z = neg
////  line(0,-REF_H,REF_B,-REF_H);
////  line(REF_B,-REF_H,REF_B,0);
////  line(REF_B,0,0,0);
///*------------------------------------------------------------------*/  

///*------------------------------------------------------------------*/  
//  /* draw acceleration vectors */
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
///*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/  
  /* draw velocity vectors */
  for (int i = 1; i < numberOfRows; i++)
  {
    stroke(255, 50, int(i * (255.0 / numberOfRows)));
    float last_dsx = velocityVectors.get(i-1)[0] * mv;
    float last_dsy = velocityVectors.get(i-1)[1] * mv;
    float last_dsz = velocityVectors.get(i-1)[2] * mv;
    float new_dsx = velocityVectors.get(i)[0] * mv;
    float new_dsy = velocityVectors.get(i)[1] * mv;
    float new_dsz = velocityVectors.get(i)[2] * mv;
    
//    line(last_dsx, last_dsy, new_dsx, new_dsy);             // 2D (x and y)
//    line(last_dsx, last_dsz, new_dsx, new_dsz);             // 2D (x and z)
//    line(last_dsy, last_dsz, new_dsy, new_dsz);             // 2D (y and z)
    line(last_dsx, last_dsy, last_dsz, new_dsx, new_dsy, new_dsz);  // 3D

//    /* draw points */
//    if (i%5 == 0)
//    {
//      pushMatrix();
//      translate(new_dsx, new_dsy, new_dsz);
//      sphere(1);
//      popMatrix();
//    }
  }
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/  
  /* draw trail vectors */
  for (int i = 1; i < numberOfRows; i++)
  {
    stroke(0, 255, int(i * (255.0 / numberOfRows)));
    float last_dsx = trailVectors.get(i-1)[0] * ms;
    float last_dsy = trailVectors.get(i-1)[1] * ms;
    float last_dsz = trailVectors.get(i-1)[2] * ms;
    float new_dsx = trailVectors.get(i)[0] * ms;
    float new_dsy = trailVectors.get(i)[1] * ms;
    float new_dsz = trailVectors.get(i)[2] * ms;
    
//    line(last_dsx, last_dsy, new_dsx, new_dsy);             // 2D (x and y)
//    line(last_dsx, last_dsz, new_dsx, new_dsz);             // 2D (x and z)
//    line(last_dsy, last_dsz, new_dsy, new_dsz);             // 2D (y and z)
    line(last_dsx, last_dsy, last_dsz, new_dsx, new_dsy, new_dsz);  // 3D

//    /* draw points */
//    if (i%5 == 0)
//    {
//      pushMatrix();
//      translate(new_dsx, new_dsy, new_dsz);
//      sphere(1);
//      popMatrix();
//    }
  }
/*------------------------------------------------------------------*/

  popMatrix();
}

void mouseWheel(MouseEvent e)
{
  translateZ -= e.getCount() * 5;
  scaleFactor += e.getCount() / 100;
}

/* my functions: */
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

double root(float n, float root)
{ 
  return Math.pow(Math.E, Math.log(n)/root);
} 
