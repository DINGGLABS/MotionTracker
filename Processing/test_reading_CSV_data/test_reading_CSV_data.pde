/* imports */
import java.util.Date;

/* defines */
int X = 500;
int Y = 500;

int NUMBER_OF_SENSOR_DATA = 8;  // t, ax, ay, az, h, Gx, Gy, Gz

/* global variables */
//float data[][];
int numberOfRows;
long t;
float ax, ay, az, ax_alt, ay_alt, az_alt, dax, day, daz;
float heading;
float Gx, Gy, Gz;

/* setup */
void setup()
{
  /* setup window */
  size(X, Y);
  //size(X, Y, P3D);
  background(255, 255, 255);
  textSize(20);
  stroke(255);
  
  /* get data */
  String dataRows[] = loadStrings("data/data.csv");
  numberOfRows = dataRows.length;
  println(dataRows.length + " total data rows"); 
  
  /* convert data to float arrays */
  float data[][] = new float[numberOfRows][NUMBER_OF_SENSOR_DATA];

  for (int i = 0; i < numberOfRows; i++)
  {
    data[i] = float(split(trim(dataRows[i]), ','));  // extract and clean (trim and split) data rows
    //println(str(data[i]));
  }
  
  /* convert unix timestamp (last 6 digits) to seconds */
  for (int i = 0; i < numberOfRows; i++)
  {
    data[i][0] /= 1000;
    //println(str(data[i]));
  }
  
  /* calculate trail vector (dsx, dsy, dsz) */
    float trailVector[][] = new float[numberOfRows][3];
  
  /* calculate traveled distance */
  for (int i = 1; i < numberOfRows; i++)
  {
    // ds = da * dt^2 = (a2 - a1) * (t2 - t1)^2
    int dt = (int)(data[i][0] - data[i-1][0]);
    trailVector[i][0] = (data[i][1] - data[i-1][1]) * dt * dt;  // dsx
    trailVector[i][1] = (data[i][2] - data[i-1][2]) * dt * dt;  // dsy
    trailVector[i][2] = (data[i][3] - data[i-1][3]) * dt * dt;  // dsz
    
    println("dsx = " + str(trailVector[i][0]) + ", " + "dsy = " + str(trailVector[i][1]) + ", " + "dsz = " + str(trailVector[i][2]));
  }
  
//blup todo: korrekte numerische Integration!!!
  

}

/* loop */
void draw()
{ 

}
 
void drawAxes(int l, int textOffset)
{
//  int tl = l + textOffset;
//  fill(255, 255, 255);
//  stroke(255, 0, 0);  // red
//  line(-l, 0, 0, l, 0, 0);
//  text("+x", l, 0, 0);
//  text("-x", -tl, 0, 0);
//  
//  stroke(0, 255, 0);  // green
//  line(0, -l, 0, 0, l, 0);
//  text("+y", 0, tl, 0);
//  text("-y", 0, -l, 0);
//  
//  stroke(0, 0, 255);  //blue
//  line(0, 0, -l, 0, 0, l);
//  text("+z", 0, 0, tl);
//  text("-z", 0, 0, -l);
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
