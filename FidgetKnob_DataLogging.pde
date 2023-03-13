/*  FidgetKnob_DataLogging written by Jack Eichenlaub, 

 This code is the accompanying Processing script for data logging from the Fidget Knob. In order to properly run the Fidget Knob, one initializes the device from starting this software (with the play button).
 
 Data output from the microcontroller of the Fidget Knob and is recorded into .txt files (in a format that can be immediately converted to .csv files).
 
 The accurate timestamp for the data is pulled directly from your system time, so make sure your computer has a proper time assigned!
 
 To mark the data at an interesting moment for later comparison with observation, hit the "m" key (after clicking into the processing 'draw" environment window).
 
 Please review the code annotations for anything that you may need to change to have the code work properly for you!
 */

import processing.serial.*;

Serial COMPort;  // Create object from Serial class
int numMax = 10000; // We are setting the maximum number of lines of data to be stored in any one .txt file. This was a redundancy measure for mis-written files.
String[] lines = new String[numMax];
String read;
String tempRead;
String tempRead2;
String lastValue;
int numLines;
int fileCounter;
long epoch = System.currentTimeMillis()/1000;  //Pulls unix timestamp from current system time of computer
int epochInt = int(epoch);                     //Converts time to integer
String nameLabel; 
String timestamp = str(epochInt) + "\n"; 
int timeFlag;

void setup()
{
  frameRate(40);                                       //This should be around or more than 2x the device output rate of around 20Hz. If you change the sample rate of the device, you might need to adjust this to maintain the 2x ratio.
  String portName = "COM16";                           //USE THE COM PORT OF YOUR OWN DEVICE HERE!!!
  COMPort = new Serial(this, portName, 115200);        //You can choose another baud rate if you desire, both 9600 and 115200 have been tested to work.
  textSize(32);
  numLines = 0;
  fileCounter = 0;
  nameLabel = "Participant1" + "_";                    //Choose your own file name (perhaps on a per-participant basis) to keep organized. Program auto generates subfiles while running
  for (int i = 0; i < 5; i++) {                        //Write the timestamp a few times to make sure it is received well
    COMPort.write(timestamp);                          //send a UNIX timestamp
    //print(timestamp);
    delay(250);
  }
}

void draw()
{
  background(51);
  if ((keyPressed == true) && (key == 'm')) {
    timeFlag = 1; 
    text(key, 20, 75); // Draw at coordinate (20,75)
  }
  if ((COMPort.available() > 0) && (numLines < numMax)) {  // If data is available,
    read = COMPort.readStringUntil('\n');                  // read and store data string in 'read' up until a new line
    if (read != null && timeFlag == 0) {                   // Case where no 'marker' has been triggered
      //print(read);                                       // Uncomment if you want to see the data being read in the Processing window.
      lines[numLines] = read.trim();                       // append new read to string lines
      numLines++;
    } 
    else if (read != null && timeFlag == 1) {              //case where 'marker' has been triggered
      //print(read);
      tempRead = read.trim();
      tempRead2 = tempRead.substring(0,(tempRead.length()-1));
      lines[numLines] = tempRead + '1';                    // append new read to string lines
      numLines++;
      timeFlag = 0;                                        //reset marker
    } 
  } 
  else if ((COMPort.available() > 0) && (numLines >= numMax)) {
    String fileName = nameLabel + fileCounter + ".txt";
    saveStrings(fileName, lines);//save string to file
    numLines = 0;
    fileCounter++;
  } 
  else {
    String fileName = nameLabel + (fileCounter+1) + ".txt";
    saveStrings(fileName, lines);//save string to file
  }
}
