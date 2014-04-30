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
 *
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

// Throttle Declarations
int throttlePin = A11; // Throttle Pot Signal
int leftMotor = DAC0;  // To Protoboard (5)
int rightMotor = DAC1;  // To Protoboard (6)
int maxThrottle = 450; // This value needs to correspond to the maximum voltage reading off of the potentiometer 
                       // plugged into the formula : maxThrottle = measuredVoltage * (3.3/(2^Res))
int Res = 10;  
int pass = 0;

// Steering Declarations
int steeringPin = A10; // Steering Pot Signal
int steeringMidPt = 487;  

// Encoder Declarations
 int val; 
 int encoder0PinA = 23; // Encoder A2
 int encoder0PinB = 25; // Encoder A1
 int encoder1PinA = 27; // Encoder B2
 int encoder1PinB = 29; // Encoder B1
 
 int encoderSampleTime = 500;  //Sample time for the encoders (in milliSeconds)
 int RPM_0 = 0;
 int RPM_1 = 0;
 
 // Extra Pins
 int SDPin = 10;
 int PullDownPin = 9;          // Press Sensor pin
 int LEDPin = 8;          //Red LED in digital pin 8 with resistor in series.
//------------------------------------------------------------------------------------------

void setup() {
  // Setup encoder inputs
  pinMode (encoder0PinA,INPUT);
  pinMode (encoder0PinB,INPUT);
  pinMode (encoder1PinA,INPUT);
  pinMode (encoder1PinB,INPUT);
  Serial.begin (9600); // Start serial output for debugging
}

void loop() {
  
  readEncoders();  //This function has no output because it changes global variables within the function.
  
 // This section of the code uses the throttle code by Dakota to drive the cart from the throttle potentiometer

  analogReadResolution(Res);
  int throttle_in = analogRead(throttlePin);
  
  throttle_in = throttleFilter(throttle_in, pass);
  
  analogReadResolution(Res);
  int Steering = analogRead(steeringPin);
  
      #if DEBUG
         #if USER_INPUT  
        Serial.print("Gas: ");                            //Debug Statements ~ visual data
        Serial.println(throttle_in);
        Serial.print("Steering");
        Serial.println(Steering);
        #endif
      #endif  
  
  int leftSpeed  = map(throttle_in, 0, maxThrottle, 0, 1023);
  int rightSpeed = map(throttle_in, 0, maxThrottle, 0, 1023);
    
  analogWriteResolution(Res);
  analogWrite(leftMotor, leftSpeed);
  analogWriteResolution(Res);
  analogWrite(rightMotor, rightSpeed);

      #if DEBUG
        #if SPEED
        Serial.print("Left Motor Voltage ");                //Debug Statements ~ visual data
        Serial.println(leftSpeed);  
        Serial.print("Right Motor Voltage");
        Serial.println(rightSpeed);
        #endif
      #endif
  if(pass == 0){
    pass = 1;
  }    
  delayMicroseconds(5);    
}


/****************************************************************************************
 ****************************************************************************************
 * Functions are defined below:
 ****************************************************************************************
 ****************************************************************************************/



void readEncoders(){
  // This section of the code reads the inputs to the wheel encoders
  int encoder0Pos = 0;
  int encoder1Pos = 0;
 
  int encoder0PinALast = LOW;
  int encoder1PinALast = LOW;
  
  int n = LOW;
  int m = LOW;

  uint32_t time;
  
   time=millis();
     while(millis()<(time + encoderSampleTime)){
     
       n = digitalRead(encoder0PinA);
       if ((encoder0PinALast == LOW) && (n == HIGH)) {
         if (digitalRead(encoder0PinB) == LOW) {
           encoder0Pos--;
         } else {
           encoder0Pos++;
         }
       }
      m = digitalRead(encoder1PinA);
       if((encoder1PinALast == LOW) && (m == HIGH)) {
        if(digitalRead(encoder1PinB) == LOW) {
         encoder1Pos--;
        } else {
           encoder1Pos++;
        }
       } 
       RPM_0 = ((encoder0Pos*(1000/encoderSampleTime))*60)/90;
       RPM_1 = ((encoder1Pos*(1000/encoderSampleTime))*60)/90;
       
       encoder0PinALast = n; 
       encoder1PinALast = m;
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

//***************************************************************************************************************************

int throttleFilter(int throttle, int pass){
  int deadband;
    int i;
    int throttle_array[10];
    int total;
    int avg_throttle;
    int shift;        
    int tolerance;
  
  if(pass == 0){       // This means that on the first pass, declare these variables in this loop, then leave them be from then on
    deadband = 2;
    total = 10;
    avg_throttle = 1;
    shift = 10;        // This shift is to account for low end values and their lack of response to the tolerance
    tolerance = 10;    /* This is the filtering tolerance.  This number represents the magnitude of increase allowable
                        * between consecutively read values.  Increase tolerance to allow faster pedal response (at 
                        * the cost of letting larger throttle noise spikes in).
                        */
    for(i = 0; i<10 ; i++){
       throttle_array[i] = 1;
    }      
    i = 0;                    
  }
  
  throttle_array[i] = (analogRead(throttle) + 1);
  
  Serial.println(throttle_array[i]);
  
  if(i != 0){
    if(throttle_array[i] > ((tolerance*throttle_array[i-1]) + shift)){
      throttle_array[i] = throttle_array[i-1];
    }
  } else {
    if(throttle_array[i] > ((tolerance*throttle_array[9]) + shift)){
      throttle_array[i] = throttle_array[9];
    }
  }
 
  total = throttle_array[0] + throttle_array[1] + throttle_array[2] + throttle_array[3] + throttle_array[4] + 
          throttle_array[5] + throttle_array[6] + throttle_array[7] + throttle_array[8] + throttle_array[9];
 
  avg_throttle = total /  10;
  
 /* if(avg_throttle < 1){
    avg_throttle = 1;
  }*/
  
  i++;
  
  if( i > 9){
    i = 0; 
  }
  
  return(throttle);
  
}
