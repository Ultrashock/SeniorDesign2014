/*****************************************************************************************
 * Senior Design Project: Electronic "Soft Differential"
 * Credit: UNCA-NCSU Senior Class 2014
 * Purpose: The purpose of this program is to read in various inputs from two wheel speed
 *          encoders, one steering potentiometer, and one throttle potentiometer.  The 
 *          code will determine if the car is straight or turning, and by how much it is
 *          turning.  An algorithm will process this and send outputs to two electric
 *          motors which will run at different speeds in a turn to simulate what is 
 *          commonly seen in a regular mechanical differential.
 *  
 * Versions & Version Notes:
 *   v0.1.0- Code written for analog input of potentiometer inputs for steering and throttle
 *      Credit: Dakota Lazenby
 *   v0.1.1- Added support for motor encoders and modified structure
 *      Credit: Dakota Lazenby
 *   v1.0.0- Ported code over for Arduino Due and added throttle response functionality
 *      Credit: Dakota Lazenby
 *   v1.0.1- Imported simplified working motor encoder code
 *      Credit: Brandon Zschokke, Jennifer Cory, and Hallie Sheaffer
 *   v2.0.0- Combined and created confirmed working throttle code and encoder code
 *      Credit: Brandon Zschokke
 *   v2.1.0- Re-Located the encoder code to a function, cleaned up some variables and added conditional prints
 *      Credit: Dakota Lazenby
 *   v2.2.0- Added in a throttleFilter function.
 *      Credit: Dakota Lazenby
 *   v2.3.0- Added functionality for power LED, checking pack voltage from LiPo battery, new encoder Z pin reads
 *            at high RPMs, 
 *      Credit: Dakota Lazenby, Hallie Shaeffer
 *    
 * Future Versions / Roadmap:
 *   v2.x.x- Implement Hallie's SD Card code for data logging. Implement Steering Code
 *   v3.0.0- Implement controller code
 *   v3.x.x- Debug and refine control code
 *   v4.0.0- Remove excess global variables and modularize code into various functions   
 *****************************************************************************************/

#define DEBUG      0
#define USER_INPUT 0
#define ENCODERS   0
#define SPEED      0
// Global Declaration Section

int control = 0;

// Throttle Declarations
int throttlePin = A11; // Throttle Pot Signal
int leftMotor = DAC0;  // To Protoboard (5)
int rightMotor = DAC1;  // To Protoboard (6)
int minThrottle = 7;
int maxThrottle = 235; // This value needs to correspond to the maximum voltage reading off of the potentiometer 
                       // plugged into the formula : maxThrottle = measuredVoltage * (3.3/(2^Res))                       
int Res = 10;  // Resolution
int noiseDelay = 10; //Delay to ensure that analogReads are performed correctly

// Steering Declarations
int steeringPin = A10; // Steering Pot Signal
int steeringMax = 600;   //THESE VALUES NEED TO BE MEASURED AND ENTERED CORRECTLY
int steeringMidPt = 400;  //IDEALLY WE WRITE CALIBRATION CODE FOR THESE VALUES
int steeringMin = 200;
int Lr = 35;                    //35 inches from center to center on rear wheels 

// Encoder Declarations 
 //CHECK THESE PINOUTS!!!!!
 int encoder0PinA = 25; // Encoder A2
 int encoder0PinB = 29; // Encoder A1
 int encoder0PinZ = 33;
 int encoder1PinA = 23; // Encoder B2
 int encoder1PinB = 27; // Encoder B1
 int encoder1PinZ = 31;
 int maxRPM = 500;
 int RPM_0_Last = 0;
 int RPM_1_Last = 0;
 
 //PID control Variables
 int Kp = 10;
 int Ki = 1;
 int Kd = 0;
 long loopStart = 0;
 long loopEnd = 0;
 long dT = 0;
 
 int control = 0;
 int error = 0;
 int controlPrev = 0;
 int errorPrev = 0;

 //Encoder read variables
 int encoderSampleTime = 125;  //Sample time for the encoders (in milliSeconds)
 int RPM_0 = 0;
 int RPM_1 = 0;
 
 //Pack voltage variables
 int minPackVoltage = 720;  //Integer value corresponding to roughly 9.65 volts (safe operating LiPo voltage)
 
 // Extra Pins
 int PwrIn = A9;  //Pin to read the battery pack nominal voltage
 int LEDPwr = 11; //Pin to control the "power to arduino" LED
 int SDPin = 10;
 int PullDownPin = 9;          // Press Sensor pin
 int LEDPin = 8;          //Red LED in digital pin 8 with resistor in series.
//------------------------------------------------------------------------------------------



/****************************************************************************************
 ****************************************************************************************
 * Functions are defined below:
 ****************************************************************************************
 ****************************************************************************************/
 
void readEncodersAB(){
  // This section of the code reads the inputs to the wheel encoders
  //This section can determine direction of rotation as well as speed
  int encoder0Pos = 0;
  int encoder1Pos = 0;
 
  int encoder0PinALast = LOW;
  int encoder1PinALast = LOW;
  
  int m = LOW;
  int n = LOW;

  uint32_t time = millis();
  
     while(millis() < (time + encoderSampleTime)){
     
       m = digitalRead(encoder0PinA);
       if ((encoder0PinALast == LOW) && (n == HIGH)) {
         if (digitalRead(encoder0PinB) == LOW) {
           encoder0Pos--;
         } else {
           encoder0Pos++;
         }
       }
      n = digitalRead(encoder1PinA);
       if((encoder1PinALast == LOW) && (m == HIGH)) {
        if(digitalRead(encoder1PinB) == LOW) {
         encoder1Pos--;
        } else {
           encoder1Pos++;
        }
       } 
       
       encoder0PinALast = m; 
       encoder1PinALast = n;
       
       RPM_0 = ((encoder0Pos*(1000/encoderSampleTime))*60)/400;
       RPM_1 = ((encoder1Pos*(1000/encoderSampleTime))*60)/400;    
   }
   
        #if DEBUG
          #if ENCODERS 
          Serial.print("RPM Left Wheel");                            //Debug Statements ~ visual data
          Serial.println(RPM_0);
          Serial.print("RPM Right Wheel");
          Serial.println(RPM_1);
          #endif
        #endif
  
    return;
}


//*****************************************************************************************************
void readEncodersZ_hallie(){
   //This section reads the z_pulse from the encoder because it only happens 1/rev
   //This section is used for RPM greater than the max value that can be read using quadriture
   //This section is uni- directional meaning it can only read speed and NOT direction of turn
   
   // Serial.println("Encoder speed over 500 RPM");
   
   //TIME VARIABLES MAY NEED TO BE uint32_t or long int NOT SURE
     
     int Z_Rev_0 = 0;
     int Z_Rev_1 = 0;
     long time00 = micros();     //Take a timestamp
       while ((Z_Rev_0 >= 0) && (Z_Rev_0 < 10)) {     // Turn 10 Revolutions
          if ((digitalRead(encoder0PinZ)) == LOW) {  //Test for high pulse
            while ((digitalRead(encoder0PinZ)) == LOW) {}  //Wait for a pulse
              Z_Rev_0++;
           } 
       }
       long time01 = micros() - time0;
 
     long time10 = micros();     //Take a timestamp
       while ((Z_Rev_1 >= 0) && (Z_Rev_1 < 10)) {     // Turn 10 Revolutions
          if ((digitalRead(encoder1PinZ)) == LOW) {  //Test for high pulse
            while ((digitalRead(encoder1PinZ)) == LOW) {}  //Wait for a pulse
              Z_Rev_1++;
           } 
       }
       long time11 = micros() - time10;
       RPM_0 = (Z_Rev_0 + 1)*1000*1000*60/time01;
       RPM_1 = (Z_Rev_1 + 1)*1000*1000*60/time11;
       
}

void readEncodersZ_hallie_var2(){
   //This section reads the z_pulse from the encoder because it only happens 1/rev
   //This section is used for RPM greater than the max value that can be read using quadriture
   //This section is uni- directional meaning it can only read speed and NOT direction of turn
   
   // Serial.println("Encoder speed over 500 RPM");
   
   //TIME VARIABLES MAY NEED TO BE uint32_t or long int NOT SURE
     
     int Z_Rev_0 = 0;
     int Z_Rev_1 = 0;
     if((digitalRead(encoder0PinZ)) == HIGH){  //WAIT FOR INITIAL PULSE
       long time00 = micros();     //Take a timestamp
     } else {
       while ((digitalRead(encoder0PinZ)) == LOW) {}  //Wait for a pulse
       long time00 = micros();     //Take a timestamp
     }
       while ((Z_Rev_0 >= 0) && (Z_Rev_0 < 10)) {     // Turn 10 Revolutions
          if ((digitalRead(encoder0PinZ)) == LOW) {  //Test for high pulse
            while ((digitalRead(encoder0PinZ)) == LOW) {}  //Wait for a pulse
              Z_Rev_0++;
           } else {
             while ((digitalRead(encoder0PinZ)) == HIGH) {}  //Wait for a pulse
             while ((digitalRead(encoder0PinZ)) == LOW) {}  //Wait for a pulse
              Z_Rev_0++;
           }
       }
       long time01 = micros() - time0;
       
     //REPEAT FOR SECOND ENCODER
     if((digitalRead(encoder1PinZ)) == HIGH){  // WAIT FOR INITIAL PULSE
       long time10 = micros();     //Take a timestamp
     } else {
       while ((digitalRead(encoder1PinZ)) == LOW) {}  //Wait for a pulse
       long time10 = micros();     //Take a timestamp
     }
       while ((Z_Rev_1 >= 0) && (Z_Rev_1 < 10)) {     // Turn 10 Revolutions
          if ((digitalRead(encoder1PinZ)) == LOW) {  //Test for high pulse
            while ((digitalRead(encoder1PinZ)) == LOW) {}  //Wait for a pulse
              Z_Rev_1++;
           } else {
             while ((digitalRead(encoder1PinZ)) == HIGH) {}  //Wait for a low
             while ((digitalRead(encoder1PinZ)) == LOW) {}  //Wait for a pulse
              Z_Rev_1++;
           }
       }
       long time11 = micros() - time10;
       
       RPM_0 = (Z_Rev_0 + 1)*1000*1000*60/time01;
       RPM_1 = (Z_Rev_1 + 1)*1000*1000*60/time11;
}
       
void readEncodersZ(){ 
   //This section reads the z_pulse from the encoder because it only happens 1/rev
   //This section is used for RPM greater than the max value that can be read using quadriture
   //This section is uni- directional meaning it can only read speed and NOT direction of turn  
  
     unsigned long int deltaTime0 = 0;
     unsigned long int deltaTime1 = 0;
     uint32_t time01;
     uint32_t time02;
     uint32_t time11;
     uint32_t time12;
     
      for(int j = 0; j < 10; j++){
       //READ Z PIN FOR ENCODER 0
       if((digitalRead(encoder0PinZ)) == HIGH){
         time01 = micros();
         while((digitalRead(encoder0PinZ)) == HIGH){} //Just wait and poll
         while((digitalRead(encoder0PinZ)) == LOW){} //Just wait and poll
         time02 = micros() - time01;
       } else {
         while((digitalRead(encoder0PinZ)) == LOW){} //Just wait and poll
         time01 = micros();
         while((digitalRead(encoder0PinZ)) == HIGH){} //Just wait and poll
         while((digitalRead(encoder0PinZ)) == LOW){} //Just wait and poll
         time02 = micros() - time01;
       }
       //REPEAT FOR ENCODER 1
       if((digitalRead(encoder1PinZ)) == HIGH){
         time11 = micros();
         while((digitalRead(encoder1PinZ)) == HIGH){} //Just wait and poll
         while((digitalRead(encoder1PinZ)) == LOW){} //Just wait and poll
         time12 = micros() - time11;
       } else {
         while((digitalRead(encoder1PinZ)) == LOW){} //Just wait and poll
         time11 = micros();
         while((digitalRead(encoder1PinZ)) == HIGH){} //Just wait and poll
         while((digitalRead(encoder1PinZ)) == LOW){} //Just wait and poll
         time12 = micros() - time11;
       } 
       
       deltaTime0 = deltaTime0 + time02;
       deltaTime1 = deltaTime1 + time12;
     }
     
     RPM_0 = 10*1000*1000*60/deltaTime0;  //nRotations*uSec2milliSec*milliSec2Sec*Sec2Min/cumulativeTimeIn_uSec
     RPM_1 = 10*1000*1000*60/deltaTime1;  //nRotations*uSec2milliSec*milliSec2Sec*Sec2Min/cumulativeTimeIn_uSec
     
}

//*****************************************************************************************************
//*****************************************************************************************************

int packVoltage(){
    int Voltage = analogRead(PwrIn);
    delay(ADC_DELAY);
    if(Voltage < minPackVoltage) {
       while(1){  
         //Infinite Loop Halting all other processes other than indicating low voltage (LED Blinks)
         digitalWrite(LEDPwr, HIGH);
         delay(500);
         digitalWrite(LEDPwr, LOW);
         delay(500);
       }
    }
}
  
//*****************************************************************************************************  
//*****************************************************************************************************

void differential(int Steering, int throttle, int &RSpeed, int &LSpeed){
  //UNFINISHED, NEED TO WORK OUT EQUATIONS FOR Ri AND THEN WORK OUT GOOD CODE FOR DIFFERENTIAL
  //THESE EQUATIONS ARE EXTREMELY RUDIMENTARY AND NEED TUNING BUT THEY SHOULD "WORK"...
    int PT, PS;
    int IV, OV, Ri;
    int Max = maxThrottle;
    int Ri;  //Need Ri in inches
    PT = throttle/maxThrottle;    
    int steerDifference = Steering - steeringMidpt;
    PS = (steerDifference)/steeringMidpt;  //Normalizes the steering angle as a percentage.
    PS = abs(PS);
    if(steerDifference < 0){
      Ri = minTurnRadius*((steeringMidpt - steeringMin)/(steeringMidpt - Steering));
    } else if(steerDifference > 0){
      Ri = minTurnRadius*((steeringMax - steeringMidpt)/(steeringMax - Steering));
    } else {
      Ri = 35000; // This value is "infinity" because the radius of curvature for a straight line is infinity
    }
      IV = (2*Ri*PT*Max)/(2*Ri+Lr);
      OV = (2*PT*Max)/(1+(Ri/(Ri+Lr)));
      
      if((Steering - steeringMidpt) < 0){
        RSpeed = OV;
        LSpeed = IV;   
      } else {  
        RSpeed = IV;
        LSpeed = OV;
      }
    
}

//*****************************************************************************************************  
//*****************************************************************************************************

void PID() { //BUILD THIS HERE BUT...MAKE ALL VARIABLES GLOBAL AND STICK THIS IN THE MAIN LOOP, DONT CALL AS FUNCTION
   //NOT READY FOR PRODUCTION
   dT = loopEnd - loopStart;
   error = readValue - offest;
   control = (Kp*error)+(Ki*(((error+errorPrev)/2)*dT+controlPrev))+(Kd*((error-errorPrev)/dT);
   errorPrev = error;
   controlPrev = control;
}

//*****************************************************************************************************  
//*****************************************************************************************************

void calibrateThrottle () {
   //This function establishes min and max throttle values______________________________
  pinMode (PullDownPin, INPUT);
  digitalWrite(PullDownPin, HIGH);
  pinMode(LEDPin, OUTPUT);
  while (digitalRead(PullDownPin) == HIGH)  {
   digitalWrite(LEDPin, LOW);
   delay(500);
   digitalWrite(LEDPin, HIGH);
   delay(500); 
  }
  analogReadResolution(Res);
  minThrottle = analogRead(throttle);
  delay(ADC_DELAY);
  minThrottle = analogRead(throttle);
  delay(ADC_DELAY);
//  Serial.print("minThrottle = ");
//  Serial.print(minThrottle);
  digitalWrite(LEDPin, LOW);
  delay(1000);
  while (digitalRead(PullDownPin) == LOW)  {
   digitalWrite(LEDPin, LOW);
   delay(500);
   digitalWrite(LEDPin, HIGH);
   delay(500); 
  }
  analogReadResolution(Res);
  maxThrottle = analogRead(throttle);
  delay(ADC_DELAY);
  maxThrottle = analogRead(throttle);  
  delay(ADC_DELAY);
  maxThrottle = (maxThrottle + 10);
//  Serial.print(".......maxThrottle = ");
//  Serial.println(maxThrottle);
  digitalWrite(LEDPin, LOW);
  delay(500);
}

//*****************************************************************************************************  
//*****************************************************************************************************

void calibrateSteering () {
   //This function establishes min and max throttle values______________________________
  pinMode (PullDownPin, INPUT);
  digitalWrite(PullDownPin, HIGH);
  pinMode(LEDPin, OUTPUT);
  while (digitalRead(PullDownPin) == HIGH)  {
   digitalWrite(LEDPin, LOW);
   delay(200);
   digitalWrite(LEDPin, HIGH);
   delay(200); 
  }
  analogReadResolution(Res);
  steeringMin = analogRead(steeringPin);  //Pull steering wheel full left
  delay(ADC_DELAY);
  steeringMin = analogRead(steeringPin);
  delay(ADC_DELAY);
//  Serial.print("minThrottle = ");
//  Serial.print(minThrottle);
  digitalWrite(LEDPin, LOW);
  delay(1000);
  while (digitalRead(PullDownPin) == LOW)  {
   digitalWrite(LEDPin, LOW);
   delay(200);
   digitalWrite(LEDPin, HIGH);
   delay(200); 
  }
  analogReadResolution(Res);
  steeringMax = analogRead(steeringPin);  //Pull steering wheel full right
  delay(ADC_DELAY);
  steeringMax = analogRead(steeringPin);  
  delay(ADC_DELAY);
//  Serial.print(".......maxThrottle = ");
//  Serial.println(maxThrottle);
  while (digitalRead(PullDownPin) == HIGH)  {
   digitalWrite(LEDPin, LOW);
   delay(75);
   digitalWrite(LEDPin, HIGH);
   delay(75); 
  }
  while (digitalRead(PullDownPin) == LOW)  {
   digitalWrite(LEDPin, HIGH);
   delay(250); 
  }
  analogReadResolution(Res);
  steeringMidpt = analogRead(steeringPin);
  delay(ADC_DELAY);
  steeringMidpt = analogRead(steeringPin);  
  delay(ADC_DELAY);
  digitalWrite(LEDPin, LOW);
  delay(500);
  steeringMidpt = (steeringMidpt + (steeringMax/2))/2;
}


//*****************************************************************************************************
//*****************************************************************************************************


void setup() {
  // Setup encoder inputs
  pinMode (encoder0PinA,INPUT);
  pinMode (encoder0PinB,INPUT);
  pinMode (encoder0PinZ,INPUT);
  pinMode (encoder1PinA,INPUT);
  pinMode (encoder1PinB,INPUT);
  pinMode (encoder1PinZ,INPUT);
  pinMode (LEDPwr, OUTPUT);
  digitalWrite(LEDPwr, HIGH);
  Serial.begin (9600); // Start serial output for debugging
  calibrateThrottle();
  calibrateSteering();
}

void loop() {
  
  packVoltage();  //Check the battery pack to see if it is within safe operating voltage
  
  
  if ((RPM_0_Last < maxRPM) || (RPM_1_Last < maxRPM)){
    readEncodersAB();  //This function has no output because it changes global variables within the function.
                       // Read Encoder pins A and B if RPM is in "low range" (Reads for both encoders)
  } else {
    readEncodersZ();   // Read Encoder pin Z if RPM is in "high range" (Reads for both encoders)
  }
  
  RPM_0_Last = RPM_0;
  RPM_1_Last = RPM_1;
  
 // This section of the code uses the throttle code by Dakota to drive the cart from the throttle potentiometer

  analogReadResolution(Res);
  int throttle_in = analogRead(throttlePin);
  delay(ADC_DELAY);   // Delays after analogRead to allow capacitors to charge and cancels cross talk between reads 
  int throttle_in = analogRead(throttlePin);                    
                       
  //throttle_in = throttleFilter(throttle_in, pass);  // Calls the throttle filtering code (may not need this)
  
  
  analogReadResolution(Res);
  int Steering = analogRead(steeringPin);
  delay(ADC_DELAY);
  int Steering = analogRead(steeringPin);
  
      #if DEBUG
         #if USER_INPUT  
        Serial.print("Gas: ");                            //Debug Statements ~ visual data
        Serial.println(throttle_in);
        Serial.print("Steering");
        Serial.println(Steering);
        #endif
      #endif  
  
  //(insert diff code here)

  
  int throttle_in_left = throttle_in;
  int throttle_in_right = throttle_in;
  if(control == 1){
      differential(Steering, throttle_in, throttle_in_right, throttle_in_left);q
  }
  
  int leftSpeed  = map(throttle_in_left, minThrottle, maxThrottle, 0, 1023);
  int rightSpeed = map(throttle_in_right, minThrottle, maxThrottle, 0, 1023);
  
  if( leftSpeed < 5 ){  // Error checking code to ensure we do not have throttle spikes at the extremities
    leftSpeed = 0;
  }
  if ( leftSpeed > 1023 ){
    leftSpeed = 1023;
  }
  if ( rightSpeed < 5 ){
    rightSpeed = 0;
  }
  if( rightSpeed > 1023 ){
    rightSpeed = 1023;
  } 
    
  analogWriteResolution(Res);
  analogWrite(leftMotor, leftSpeed);
  delay(ADC_DELAY/5);
  analogWriteResolution(Res);
  analogWrite(rightMotor, rightSpeed);
  delay(ADC_DELAY/5);

      #if DEBUG
        #if SPEED
        Serial.print("Left Motor Voltage ");                //Debug Statements ~ visual data
        Serial.println(leftSpeed);  
        Serial.print("Right Motor Voltage");
        Serial.println(rightSpeed);
        #endif
      #endif

  delayMicroseconds(5);    
}


