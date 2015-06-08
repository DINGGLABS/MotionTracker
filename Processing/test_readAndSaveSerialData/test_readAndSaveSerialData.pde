/* imports */
import processing.serial.*;

/* defines */
int BAUDRATE = 9600;
int SERIAL_PORT = 5;
boolean firstContact = false;
String s = "knock";

/* global variables */
Serial myPort;
String stringVal;

Table table;
Table dataTable;
int numReadings = 10; //keeps track of how many readings you'd like to take before writing the file. 
int readingCounter = 0; //counts each reading to compare to numReadings. 
String fileName;

void setup()
{
//  size(200, 200); //make our canvas 200 x 200 pixels big
  
  //  initialize your serial port and set the baud rate to 112600
  myPort = new Serial(this, Serial.list()[SERIAL_PORT], BAUDRATE);
//  myPort.bufferUntil('\n');

  dataTable = new Table();

  dataTable.addColumn("id");
  
  //the following adds columns for time. You can also add milliseconds. See the Time/Date functions for Processing: https://www.processing.org/reference/ 
  dataTable.addColumn("year", Table.INT);
  dataTable.addColumn("month", Table.INT);
  dataTable.addColumn("day", Table.INT);
  dataTable.addColumn("hour", Table.INT);
  dataTable.addColumn("minute", Table.INT);
  dataTable.addColumn("second", Table.INT);
  
  //the following are dummy columns for each data stringValue. Add as many columns as you have data stringValues. Customize the names as needed. Make sure they are in the same order as the order that Arduino is sending them!
  for (int n = 0; n < 10; n++) dataTable.addColumn(str(n), Table.INT);
}

void draw()
{
  //empty
}

void serialEvent( Serial myPort)
{
  //put the incoming data into a String - 
  //the '\n' is our end delimiter indicating the end of a complete packet
  stringVal = myPort.readStringUntil('\n');
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
        myPort.write(s);
        println("contact established!");
      }
    }
    else
    { //if we've already established contact, keep getting and parsing data
      int sensorVals[] = int(split(stringVal, ','));  //parses the packet from Arduino and places the stringValues into the sensorVals array
      
      TableRow newRow = dataTable.addRow(); //add a row for this new reading
      newRow.setInt("id", dataTable.lastRowIndex());//record a unique identifier (the row's index)
    
      //record data
      newRow.setInt("year", year());
      newRow.setInt("month", month());
      newRow.setInt("day", day());
      newRow.setInt("hour", hour());
      newRow.setInt("minute", minute());
      newRow.setInt("second", second());
      
      //record sensor information. Customize the names so they match your sensor column names.
      for (int n = 0; n < 10; n++) newRow.setInt(str(n), sensorVals[n]);
      
      readingCounter++;
      
      //saves the table as a csv in the same folder as the sketch every numReadings. 
      if (readingCounter % numReadings == 0)//The % is a modulus, a math operator that signifies remainder after division. The if statement checks if readingCounter is a multiple of numReadings (the remainder of readingCounter/numReadings is 0)
      {
        fileName = str(year()) + str(month()) + str(day()) + "_" + str(dataTable.lastRowIndex()-readingCounter+1); //this filename is of the form year+month+day+readingCounter
        saveTable(dataTable, "data/" + fileName + ".csv"); //Woo! save it to your computer. It is ready for all your spreadsheet dreams. 
      }
    }
      
  
//      if (mousePressed == true) 
//      {                           //if we clicked in the window
//        myPort.write('1');        //send a 1
//        println("1");
//      }
  
//      // when you've parsed the data you have, ask for more:
//      myPort.write(s);
  }
  //else if (!firstContact) println("no data aviable");
}
