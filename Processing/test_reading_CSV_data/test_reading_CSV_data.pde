/* imports */
import java.util.Date;

/* defines */
int X = 600;
int Y = 600;

int NUMBER_OF_SENSOR_DATA = 8;  // t, ax, ay, az, h, Gx, Gy, Gz

/* global variables */
int mX, mY;    // mouse coordinates

int numberOfRows;
ArrayList<float[]> data = new ArrayList();                // global generic array list
ArrayList<float[]> accelerationVector = new ArrayList();  // global generic array list
ArrayList<float[]> velocityVector = new ArrayList();      // global generic array list
ArrayList<float[]> trailVector = new ArrayList();         // global generic array list

/* setup */
void setup()
{
  /* get data */
  String dataRows[] = loadStrings("data/data.csv");
  numberOfRows = dataRows.length;
  //float data[][] = new float[numberOfRows][NUMBER_OF_SENSOR_DATA];
  //float trailVector[][] = new float[numberOfRows][3];

  /* setup window */
  //size(X, Y);
  size(X, Y, P3D);
  //background(255, 255, 255);
  textSize(20);
  stroke(255);
    
  /* print total number of rows */
  println(dataRows.length + " total data rows"); 
  
  /* convert string data to float arrays */
  for (int i = 0; i < numberOfRows; i++)
  {
    data.add(float(split(trim(dataRows[i]), ',')));  // extract and clean (trim and split) data rows
    println(str(data.get(i)));
  }
  
  /* convert unix timestamp (last 6 digits) to seconds */
  for (int i = 0; i < numberOfRows; i++)
  {
    data.get(i)[0] /= 1000;
    //println(str(data.get(i)));
  }
  
  /* get acceleration vectors */
  for (int i = 0; i < numberOfRows; i++)
  {
    float a[] = {data.get(i)[1], data.get(i)[2], data.get(i)[3]};
    accelerationVector.add(i, a);
  }
  
  /* rotate acceleration vectors to fit them to the room coordinate system */
  for (int i = 0; i < numberOfRows; i++)
  {
    /* collect current data */
    float roll = data.get(i)[5];     // x-rotation angle
    float pitch = data.get(i)[6];    // y-rotation angle
    float heading = data.get(i)[7];  // z-rotation angle
    
    float ax = accelerationVector.get(i)[0];
    float ay = accelerationVector.get(i)[1];
    float az = accelerationVector.get(i)[2];
    
    float w;  // current angle
    
    /* x-axis rotation: */
    w = PI/180*roll;
    ay = ay * cos(w) - az * sin(w);
    az = ay * sin(w) + az * cos(w);
    
    /* y-axis rotation: */
    w = PI/180*pitch;
    ax = ax * cos(w) + az * sin(w);
    az = -ax * sin(w) + az * cos(w);
    
    /* z-axis rotation: */
    w = PI/180*heading;
    ax = ax * cos(w) - ay * sin(w);
    ay = ax * sin(w) + ay * cos(w);
    
    float a[] = {ax, ay, az};
    accelerationVector.set(i, a);
  }
  
  /* remove the g-vectors from the acceleration vectors */
  for (int i = 0; i < numberOfRows; i++)
  {
    accelerationVector.get(i)[0] += 9.81 * sin o. cos???;
    accelerationVector.get(i)[1] += 9.81 * sin o. cos???;
    accelerationVector.get(i)[2] += 9.81 * sin o. cos???;
    println(str(accelerationVector.get(i)));
  }
  
  
  /* calculate velocity and trail vectors */
  float zeros[] = {0, 0, 0};
  velocityVector.add(0, zeros);  // init first position with zeros
  trailVector.add(0, zeros);     // init first position with zeros 
  
  for (int i = 1; i < numberOfRows; i++)
  {
    /* get delta t */
    float dt = data.get(i)[0] - data.get(i-1)[0];
    //println(dt);
   
    /* dv = da * dt */
    float dvx = (accelerationVector.get(i)[0] - accelerationVector.get(i-1)[0]) * dt;
    float dvy = (accelerationVector.get(i)[1] - accelerationVector.get(i-1)[1]) * dt;
    float dvz = (accelerationVector.get(i)[2] - accelerationVector.get(i-1)[2]) * dt;
    float v[] = {dvx, dvy, dvz}; 
        
    /* add last velocity to current one */
    for (int j = 0; j < 3; j++)
    {
      v[j] += velocityVector.get(i-1)[j];
    }
    
    /* v_neu = dv + v_old */
    velocityVector.add(i, v);
   
    /* ds = dv * dt */
    float dsx = velocityVector.get(i)[0] * dt;
    float dsy = velocityVector.get(i)[1] * dt;
    float dsz = velocityVector.get(i)[2] * dt;
    float s[] = {dsx, dsy, dsz};
        
    /* add last trail to current one */
    for (int j = 0; j < 3; j++)
    {
      s[j] += trailVector.get(i-1)[j];
    }
    
    /* s_neu = ds + s_old */
    trailVector.add(i, s);
    
    
//    /* numeric integration: trapezoidal rule: ds = (a2 + a1)/2 * (t2 - t1)^2 */
//    float dsx = (data.get(i)[1] + data.get(i-1)[1])/2 * dt * dt;
//    float dsy = (data.get(i)[2] + data.get(i-1)[2])/2 * dt * dt;
//    float dsz = (data.get(i)[3] + data.get(i-1)[3])/2 * dt * dt;
    
    
    //println("dsx = " + str(trailVector.get(i-1)[0]) + ", " + "dsy = " + str(trailVector.get(i-1)[1]) + ", " + "dsz = " + str(trailVector.get(i-1)[2]));
  }
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
  
  drawAxes(X/3*2, 30);
  
  /* draw reference field */
  stroke(255, 255, 255);
//  line(0,0,0,400);          // x = pos, z = pos
//  line(0,400,400,400);
//  line(400,400,400,0);
//  line(400,0,0,0);
  
//  line(0,0,0,400);          // x = neg, z = pos
//  line(0,400,-400,400);
//  line(-400,400,-400,0);
//  line(-400,0,0,0);

  line(0,0,0,-400);          // x = pos, z = neg
  line(0,-400,400,-400);
  line(400,-400,400,0);
  line(400,0,0,0);
  
  /* draw vectors */
  stroke(255, 0, 255);  // violett
  float m = 1;  // multiplicator
  
  float last_dsx = 0;
  float last_dsy = 0;
  float last_dsz = 0;
  
  /* draw lines */
  for (int i = 0; i < numberOfRows - 1; i++)
  {
    float new_dsx = last_dsx + trailVector.get(i)[0] * m;
    float new_dsy = last_dsy + trailVector.get(i)[1] * m;
    float new_dsz = last_dsz + trailVector.get(i)[2] * m;
    
//    line(last_dsx, last_dsz, new_dsx, new_dsz);             // 2D (x and z)
    line(last_dsx, last_dsy, last_dsz, new_dsx, new_dsy, new_dsz);  // 3D

    last_dsx = new_dsx;
    last_dsy = new_dsy;
    last_dsz = new_dsz;
  }
  
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
  text("+z", 0, tl, 0);
  text("-z", 0, -l, 0);
  
  stroke(0, 0, 255);  //blue
  line(0, 0, -l, 0, 0, l);
  text("+y", 0, 0, tl);
  text("-y", 0, 0, -l);
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
