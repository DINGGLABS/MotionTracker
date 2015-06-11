/* imports */
import java.util.Date;

/* defines */
int X = 600;
int Y = 600;

int REF_H = 400;
int REF_B = 150;

int NUMBER_OF_SENSOR_DATA = 8;  // t, ax, ay, az, h, Gx, Gy, Gz (in this order!)

boolean CALIBRATE = true;
int NUMBER_OF_DATA_USED_TO_CALIBRATE = 50;  // that means 5sec calibrating!

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
  String dataRows[] = loadStrings("data/data.csv");
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
  for (int i = 0; i < numberOfRows; i++)
  {
    data.get(i)[0] /= 1000;
    //println(str(data.get(i)));
  }
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/ 
  /* rotate data vectors to fit them to the room coordinate system */
  for (int i = 0; i < numberOfRows; i++)
  {
    /* collect current data */
    float a[] = {data.get(i)[1], data.get(i)[2], data.get(i)[3]};
    float roll = data.get(i)[5];     // x-rotation angle
    float pitch = data.get(i)[6];    // y-rotation angle
    float heading = data.get(i)[7];  // z-rotation angle
            
    //println(str(data.get(i)));
    
    /* rotate */
    a = rotateArrayVector(a, roll, pitch, heading);
    
    /* save rotated data */
    data.get(i)[1] = a[0];
    data.get(i)[2] = a[1];
    data.get(i)[3] = a[2];
    
    //println(str(data.get(i)));
  }
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/ 
  /* calibrate data */
  if (CALIBRATE)  // device lies flat and quiet on a plane ground for at least 5 seconds
  {
    float ax_c = 0;
    float ay_c = 0;
    float az_c = 0;
    float Gx_c = 0;
    float Gy_c = 0;
    float Gz_c = 0;
    
    /* get mean values */
    for (int i = 0; i < NUMBER_OF_DATA_USED_TO_CALIBRATE; i++)
    {
      ax_c += data.get(i)[1]; ay_c += data.get(i)[2]; az_c += data.get(i)[3];
      Gx_c += data.get(i)[5]; Gy_c += data.get(i)[6]; Gz_c += data.get(i)[7];
    }
    ax_c /= NUMBER_OF_DATA_USED_TO_CALIBRATE; ay_c /= NUMBER_OF_DATA_USED_TO_CALIBRATE; az_c /= NUMBER_OF_DATA_USED_TO_CALIBRATE;
    Gx_c /= NUMBER_OF_DATA_USED_TO_CALIBRATE; Gy_c /= NUMBER_OF_DATA_USED_TO_CALIBRATE; Gz_c /= NUMBER_OF_DATA_USED_TO_CALIBRATE;
    
    for (int i = 0; i < numberOfRows; i++)
    {
      /* recalculate data values */
      data.get(i)[1] -= ax_c; data.get(i)[2] -= ay_c; data.get(i)[3] -= az_c;  // remove offset
      if (i < 10) data.get(i)[4] = data.get(10)[4];                            // set first 10 values equal the 10th
      data.get(i)[5] -= Gx_c; data.get(i)[6] -= Gy_c; data.get(i)[7] -= Gz_c;  // remove offset
            
      //println(str(data.get(i)));
      
      /* rotate by the g-error */
      float a[] = {data.get(i)[1], data.get(i)[2], data.get(i)[3]};
      float roll = acos(1/(sqrt(ay_c*ay_c + az_c*az_c)));     // x-rotation angle ( phi = acos((a o b) / (|a| * |b|)) )
      float pitch = acos(1/(sqrt(ax_c*ax_c + az_c*az_c)));    // y-rotation angle
      float heading = acos(1/(sqrt(ax_c*ax_c + ay_c*ay_c)));  // z-rotation angle
                    
      /* rotate */
      a = rotateArrayVector(a, roll, pitch, heading);
      
      /* save rotated data */
      data.get(i)[1] = a[0];
      data.get(i)[2] = a[1];
      data.get(i)[3] = a[2];
      
      println(str(data.get(i)));
    }        
  }
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/ 
  /* set acceleration vectors */
  for (int i = 0; i < numberOfRows; i++)
  {
    float a[] = {data.get(i)[1], data.get(i)[2], data.get(i)[3]};
    accelerationVectors.add(i, a);
  }
/*------------------------------------------------------------------*/ 

/*------------------------------------------------------------------*/   
  /* calculate velocity and trail vectors */
  float zeros[] = {0, 0, 0};
  velocityVectors.add(0, zeros);  // init first position with zeros
  trailVectors.add(0, zeros);     // init first position with zeros 
  
  for (int i = 1; i < numberOfRows; i++)
  {
    /* get delta t */
    float dt = data.get(i)[0] - data.get(i-1)[0];
    //println(dt);
   
    /* dv = da * dt */
    float dvx = (accelerationVectors.get(i)[0] - accelerationVectors.get(i-1)[0]) * dt;
    float dvy = (accelerationVectors.get(i)[1] - accelerationVectors.get(i-1)[1]) * dt;
    float dvz = (accelerationVectors.get(i)[2] - accelerationVectors.get(i-1)[2]) * dt;
    float v[] = {dvx, dvy, dvz}; 
        
    /* add last velocity to current one */
    for (int j = 0; j < 3; j++)
    {
      v[j] += velocityVectors.get(i-1)[j];
    }
    
    /* v_neu = dv + v_old */
    velocityVectors.add(i, v);
   
    /* ds = dv * dt */
    float dsx = velocityVectors.get(i)[0] * dt;
    float dsy = velocityVectors.get(i)[1] * dt;
    float dsz = velocityVectors.get(i)[2] * dt;
    float s[] = {dsx, dsy, dsz};
        
    /* add last trail to current one */
    for (int j = 0; j < 3; j++)
    {
      s[j] += trailVectors.get(i-1)[j];
    }
    
    /* s_neu = ds + s_old */
    trailVectors.add(i, s);
    
    println("dsx = " + str(trailVectors.get(i-1)[0]) + ", " + "dsy = " + str(trailVectors.get(i-1)[1]) + ", " + "dsz = " + str(trailVectors.get(i-1)[2]));
  }
  
//    /* numeric integration: trapezoidal rule: ds = (a2 + a1)/2 * (t2 - t1)^2 */
//    float dsx = (data.get(i)[1] + data.get(i-1)[1])/2 * dt * dt;
//    float dsy = (data.get(i)[2] + data.get(i-1)[2])/2 * dt * dt;
//    float dsz = (data.get(i)[3] + data.get(i-1)[3])/2 * dt * dt;
/*------------------------------------------------------------------*/ 
}

/* loop */
void draw()
{
/*------------------------------------------------------------------*/  
  /* draw diagram */
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
  
  drawAxes(X/3*2, 30);
/*------------------------------------------------------------------*/  

/*------------------------------------------------------------------*/  
  /* draw reference trail */
  stroke(255, 255, 255);
//  line(0,0,0,REF_H);          // x = pos, z = pos
//  line(0,REF_H,REF_B,REF_H);
//  line(REF_B,REF_H,REF_B,0);
//  line(REF_B,0,0,0);
  
//  line(0,0,0,REF_H);          // x = neg, z = pos
//  line(0,REF_H,-REF_B,REF_H);
//  line(-REF_B,REF_H,-REF_B,0);
//  line(-REF_B,0,0,0);

  line(0,0,0,-REF_H);          // x = pos, z = neg
  line(0,-REF_H,REF_B,-REF_H);
  line(REF_B,-REF_H,REF_B,0);
  line(REF_B,0,0,0);
/*------------------------------------------------------------------*/  

/*------------------------------------------------------------------*/  
  /* draw vectors */
  stroke(255, 0, 255);  // violett
  float m = 10;  //blup multiplicator
  
  /* draw lines */
  for (int i = 1; i < numberOfRows; i++)
  {
    float last_dsx = trailVectors.get(i-1)[0] * m;
    float last_dsy = trailVectors.get(i-1)[1] * m;
    float last_dsz = trailVectors.get(i-1)[2] * m;
    float new_dsx = trailVectors.get(i)[0] * m;
    float new_dsy = trailVectors.get(i)[1] * m;
    float new_dsz = trailVectors.get(i)[2] * m;
    
//    line(last_dsx, last_dsy, new_dsx, new_dsy);             // 2D (x and y)
//    line(last_dsx, last_dsz, new_dsx, new_dsz);             // 2D (x and z)
//    line(last_dsy, last_dsz, new_dsy, new_dsz);             // 2D (y and z)
    line(last_dsx, last_dsy, last_dsz, new_dsx, new_dsy, new_dsz);  // 3D
    
    if (i%5 == 0)
    {
      pushMatrix();
      translate(new_dsx, new_dsy, new_dsz);
      sphere(1);
      popMatrix();
    }
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
  text("+z", 0, tl, 0);
  text("-z", 0, -l, 0);
  
  stroke(0, 0, 255);  //blue
  line(0, 0, -l, 0, 0, l);
  text("+y", 0, 0, tl);
  text("-y", 0, 0, -l);
}


float[] rotateArrayVector(float vec[], float xAngle, float yAngle, float zAngle)
{
  /* collect current data */
  float x = vec[0];
  float y = vec[1];
  float z = vec[2];
  float roll = xAngle * PI/180;     // x-rotation angle
  float pitch = yAngle * PI/180;    // y-rotation angle
  float heading = zAngle * PI/180;  // z-rotation angle
          
  //println(str(data.get(i)));
  
  /* x-axis rotation: */
  y = y * cos(roll) - z * sin(roll);
  z = y * sin(roll) + z * cos(roll);
  
  /* y-axis rotation: */
  x = x * cos(pitch) + z * sin(pitch);
  z = -x * sin(pitch) + z * cos(pitch);
  
  /* z-axis rotation: */
  x = x * cos(heading) - y * sin(heading);
  y = x * sin(heading) + y * cos(heading);
  
  float a[] = {x, y, z};
  return a;
  
  //println(str(accelerationVectors.get(i)));
}


//void serialEvent( Serial myPort)
//{
//  /* put the incoming data into a string */
//  stringVal = myPort.readStringUntil('\n');  //the '\n' is our end delimiter indicating the end of a complete packet
//  
//  /* alternatively put csv data into a string array */  
////  dataLines = loadStrings("data.csv");
//
////  for(int i = 0; i < dataLines.length; i++)
////  {
////    stringVal = dataLines[i];
//
//
//  
//  //make sure our data isn't empty before continuing
//  if (stringVal != null)
//  {
//    //trim whitespace and formatting characters (like carriage return)
//    stringVal = trim(stringVal);  //blup zwingend nötig für Erkennung von "knock"
//    println(stringVal);
//      
//    //look for our "knock" string to start the handshake
//    //if it's there, clear the buffer, and send a request for data
//    if (firstContact == false)
//    {
//      if (stringVal.equals(s))
//      {
//        myPort.clear();
//        firstContact = true;
//        myPort.write("send me some data now!");  // uC waits until data have been sent
//        println("contact established!");
//      }
//    }
//    else
//    { //if we've already established contact, keep getting and parsing data
//      float sensorVals[] = float(split(stringVal, ','));  //parses the packet from Arduino and places the stringValues into the sensorVals array
//      //println(sensorVals.length);
//      //println(str(sensorVals));
//
//      if (sensorVals.length == 10)
//      {
//        t = (long)sensorVals[0];
//        ax = sensorVals[1];
//        ay = sensorVals[2];
//        az = sensorVals[3];
//        heading = sensorVals[4];
//        Gx = sensorVals[5];
//        Gy = sensorVals[6];
//        Gz = sensorVals[7];
//      }
//      

//      
//      
////      /* calculate heading (0°... 360°)*/
////      float heading = 180 * atan2(Bz, By)/PI;  // use z and y since the device stands upright
////      if (heading < 0) heading += 360;
////      println(heading);
//      
////      /* calculate velocity */
////      float dt = ((float)t / 1000000);  //blup
////      float dvx = dax * dt;
////      float dvy = day * dt;
////      float dvz = daz * dt;
////      println(dax);
////      println(dt);
////      println(dax * dt);
////      print(dvx + ", ");
////      print(dvy + ", ");
////      println(dvz);
////      
////      float dsx = dvx * dt;
////      float dsy = dvy * dt;
////      float dsz = dvz * dt;
////      print(dsx + ", ");
////      print(dsy + ", ");
////      println(dsz);
//      
////      /* create new rows for the table */
////      TableRow newRow = dataTable.addRow(); //add a row for this new reading
////      newRow.setInt("id", dataTable.lastRowIndex());//record a unique identifier (the row's index)
////    
////      //record data
////      newRow.setInt("year", year());
////      newRow.setInt("month", month());
////      newRow.setInt("day", day());
////      newRow.setInt("hour", hour());
////      newRow.setInt("minute", minute());
////      newRow.setInt("second", second());
////      
////      //record sensor information. Customize the names so they match your sensor column names.
////      for (int n = 0; n < 10; n++) newRow.setInt(str(n), sensorVals[n]);
////      
////      readingCounter++;
////      
////      //saves the table as a csv in the same folder as the sketch every numReadings. 
////      if (readingCounter % numReadings == 0)//The % is a modulus, a math operator that signifies remainder after division. The if statement checks if readingCounter is a multiple of numReadings (the remainder of readingCounter/numReadings is 0)
////      {
////        fileName = str(year()) + str(month()) + str(day()) + "_" + str(dataTable.lastRowIndex()-readingCounter+1); //this filename is of the form year+month+day+readingCounter
////        saveTable(dataTable, "data/" + fileName + ".csv"); //Woo! save it to your computer. It is ready for all your spreadsheet dreams. 
////      }
//    }
//      
//  
//  
////  }
//  
//  
//  
//  
//  
////      if (mousePressed) 
////      {
////        myPort.write('1');        //send a 1
////        println("1");
////      }
//  
////      // when you've parsed the data you have, ask for more:
////      myPort.write("blup");
//  }
//  //else if (!firstContact) println("no data aviable");
//}
