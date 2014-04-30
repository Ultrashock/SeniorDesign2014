/* A simple data logger for the Arduino analog pins with optional DS1307
uses RTClib from https://github.com/adafruit/RTClib
Last edited on 2/17/14 -
This code is to gather encoder data while running the motors at a constant speed.  The idea being to compare rpms (measured with 
handheld programmer) with encoder output when given constant low to medium wheel speed.  Data saved to SD card.
You Must put micro SD card in larger card shell then into computer.
!!!!!!!!!! IMPORTANT: Disable power to SD card breakout board before removing the SD card, 
otherwise the file could be corrupted !!!!!!!!!!!!!!!!
*/
//--------------------------------------------------------

#include <SdFat.h>  //Not a standard library - must download from zip file available at:
//http://code.google.com/p/beta-lib/downloads/detail?name=SdFatBeta20130621.zip&can=2&q
#include <SdFatUtil.h>  // define FreeRam()

#define SD_CHIP_SELECT  SS  // SD chip select pin - may need to change to 10
#define USE_DS1307       0  // set nonzero to use DS1307 RTC
#define LOG_INTERVAL  1000  // mills between entries
#define SENSOR_COUNT     2  // number of analog pins to log -- optional now
#define ECHO_TO_SERIAL   1  // echo data to serial port if nonzero
#define WAIT_TO_START    1  // Wait for serial input in setup()
#define ADC_DELAY       10  // ADC delay for high impedence sensors

//Global integer to store chosen input
int analogPin = 10;
//int quit = 0;
int PullDownPin = 9;  // Press Sensor pin
int LEDPin = 8; //Red LED in digital pin 8 with resistor in series.

//Globals for analog reading:
int throttle = A11;
int outOne = DAC0;
int outTwo = DAC1;
int Res = 10;
int Speed = 0;
int maxThrottle = 670;
int encoderAPin1 = 25;
int encoderAPin2 = 23;
int encoderBPin1 = 29;
int encoderBPin2 = 27;
int u = 0; //For saving 10 samples of rpms before next line.
int debug = 1;
int data = 1;

// file system object
SdFat sd;

// text file for logging
ofstream logfile;

// Serial print stream
ArduinoOutStream cout(Serial);

// buffer to format data - makes it eaiser to echo to Serial
char buf[400];
//------------------------------------------------------------------------------
#if SENSOR_COUNT > 6
#error SENSOR_COUNT too large
#endif  // SENSOR_COUNT
//------------------------------------------------------------------------------
// store error strings in flash to save RAM
#define error(s) sd.errorHalt_P(PSTR(s))
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  pinMode(PullDownPin, INPUT);    // Pin 9 is connected to Press sensor
  digitalWrite(PullDownPin, HIGH); //Pin 9 is now high.  When pressed, pin & will be pulled LOW.
//Sense to see if PullDownPin is low before proceeding with code.
  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, HIGH); //LED indicator 
while (!Serial){}  // wait for DUE
  
  // pstr stores strings in flash to save RAM
  cout << endl << pstr("FreeRam: ") << FreeRam() << endl;


#if WAIT_TO_START
 // cout << pstr("Type any character to start\n");
 // while (Serial.read() <= 0) {}
  delay(400);  // catch Due reset problem
#endif  // WAIT_TO_START


  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  if (!sd.begin(SD_CHIP_SELECT, SPI_HALF_SPEED)) sd.initErrorHalt();
  //********Need to reserach SPI_HALF_SPEED to see if this is causing unnecessary lag.********
  

  // create a new file in root, the current working directory
  char name[] = "Encode00.CSV";

  for (uint8_t i = 0; i < 100; i++) {
    name[6] = i/10 + '0';
    name[7] = i%10 + '0';
    if (sd.exists(name)) continue;
    logfile.open(name);
    break;
  }

  if (!logfile.is_open()) error("file.open");

  cout << pstr("Logging to: ") << name << endl;
 // cout << pstr("Type any character to stop\n\n");
  
  // format header in buffer
  obufstream bout(buf, sizeof(buf));

  bout << pstr("millis");

  logfile << buf << endl;
  
  Ramp_up_speed(); //Ease the motors up to 150/1023

#if ECHO_TO_SERIAL
  cout << buf << endl;
#endif  // ECHO_TO_SERIAL
}
//------------------------------------------------------------------------------
void loop() {

   uint32_t m;
   int Encoder_A_Spd, Encoder_B_Spd;
  

  // wait for time to be a multiple of interval
  do {
    m = millis();
  } while (m % LOG_INTERVAL);

  // use buffer stream to format line
  obufstream bout(buf, sizeof(buf));

  // start with time in millis
  cout << ("\n");
  bout << m;
    delay(400);  // catch Due reset problem
  
 
  //Insert read encoder code here************************************************************
  Encoder_A_Spd = readEncoder(encoderAPin1, encoderAPin2);
  bout << ',' << pstr("Encoder A: ") << ',';
  bout << Encoder_A_Spd;
  delay(20);
  Encoder_A_Spd = readEncoder(encoderBPin1, encoderBPin2);
  bout << ',' << pstr("Encoder B: ") << ',';
  bout << Encoder_B_Spd;
  bout << (',');
  delay(20);
  
  u++;
  //Insert save encoder data to SD card here*************************************************
  if (u == 10) {
    bout << endl;
    cout << endl;
    u = 0;
  }
  logfile << buf << flush;
  
  delay(100);
  
    // log data and flush to SD
  logfile << buf << flush;
  
       // check for error
  // if (!logfile) error("write data failed");  
   //Always indicates an error - needs troubleshooting as of 2/3/14...


#if ECHO_TO_SERIAL
  cout << buf;
#endif  // ECHO_TO_SERIAL

  // don't log two points in the same millis
  if (m == millis()) delay(1);
  
  if (digitalRead(PullDownPin) == HIGH) return;  //Gather more data.  Toggle switch is open.
  
  while (1);
  
  
if (digitalRead(PullDownPin) == LOW) { 
  //Toggle switch is closed. Finish writing to file and close.
      logfile.close();
      cout << pstr("Done!");
  }
  
} //End of void loop

void Ramp_up_speed() {
  for (int k = 0; k<150; k++) {
  // reads the throttle input and outputs an analog voltage
 // analogReadResolution(Res);  Do we need this?
 // Speed = map(throttle_in, 0, maxThrottle, 0, 1024);
 // Jennifer, please verify that speed ranges from 0 to 1024.   
 //  analogWriteResolution(Res);  Do we need this?
  analogWrite(outOne, k);
//   analogWriteResolution(Res);  Do we need this?
  analogWrite(outTwo, k);  // We can comment this out and test one at a time...
//  Serial.print("DAC Output: "); //Debug Statements ~ visual data
//  Serial.println(Speed);
delay(50);
  }

}

int readEncoder(int encoderPin1, int encoderPin2){
  uint32_t time;  //Changed from unsigned long
  volatile unsigned int n;  //Changed from int per code on Arduino playground linked below.
  
  /* 
  * code comes from http://playground.arduino.cc/Main/RotaryEncoders#.UwO3jPZkK4Q
  * Send the function the encoder pins 1 & 2 (in the form pin1 is top
  * and pin2 is bottom with the red side facing you and holes to the top
  * of the encoder.
  * Reads encoder for a quarter of a second and returns the measurement in RPM.
  * Clockwise rotation is defined
  */
  
  /*  
  Also noted at the above linked arduino playground, we can set up an interrupt for reading the encoders. 
  */
  
  time = millis();
    n = 0;
    //
    while (millis() < (time + 250)){
      //Look for a low-to-high on channel 1
      if(digitalRead(encoderPin1) == HIGH) {
       //check channel 2 to see which way encoder is turning
         if (digitalRead(encoderPin2) == LOW)  n++;          //CW 
         else n--;                                           //CCW
      }
      else {  //must be a high-to-low edge on channel 1.
        //Check channel 2 to see which way encoder is turning
        if(digitalRead(encoderPin2) == HIGH) n++;            //CW 
        else n--;                                            //CCW  
      }
      
    }
        
    n = ((n*4)*60)/90; // This is the conversion from (pulses/quarterSecond) to RPM
        if(debug || data){
          Serial.print("RPM: ");
          Serial.println(n);
        }
    return n;
}
