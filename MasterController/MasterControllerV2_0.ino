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
 *    
 * Future Versions / Roadmap:
 *   v2.x.x- Implement Hallie's SD Card code for data logging. Implement Steering Code
 *   v3.0.0- Implement controller code
 *   v3.x.x- Debug and refine control code
 *   v4.0.0- Remove excess global variables and modularize code into various functions   
 *****************************************************************************************/
// Global Declaration Section

// Throttle Declarations
int throttle = A11; // Throttle Pot Signal
int outOne = DAC0;  // To Protoboard (5)
int outTwo = DAC1;  // To Protoboard (6)
int Res = 10;  
int Speed = 0;
int maxThrottle = 450; 

// Steering Declarations
int steering = A10; // Steering Pot Signal

// Encoder Declarations
 int val; 
 int encoder0PinA = 23; // Encoder A2
 int encoder0PinB = 25; // Encoder A1
 int encoder1PinA = 27; // Encoder B2
 int encoder1PinB = 29; // Encoder B1
 
 int encoder0Pos = 0;
 int encoder1Pos = 0;
 
 int encoder0PinALast = LOW;
 int encoder1PinALast = LOW;
 
 int n = LOW;
 int m = LOW;
 int RPM_A=0;
 int RPM_B=0;
 uint32_t time;
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
  // This section of the code reads the inputs to the wheel encoders
 time=millis();
   while(millis()<(time+500)){
     
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
   RPM_A=((encoder0Pos*2)*60)/90;
   RPM_B=((encoder1Pos*2)*60)/90;
   
   encoder0PinALast = n; 
   encoder1PinALast = m;
 }
  // Serial.print("RPM_A: ");
  // Serial.print(RPM_A);
  // Serial.println();
  // Serial.print("RPM_B: ");
  // Serial.print(RPM_B);
  // Serial.println();
 encoder0Pos=0;
 encoder1Pos=0;

 // This section of the code uses the throttle code by Dakota to drive the cart from the throttle potentiometer

 analogReadResolution(Res);
  int throttle_in = analogRead(throttle);
  Serial.print("Throttle input: ");                            //Debug Statements ~ visual data
  Serial.println(throttle_in);
  
  Speed = map(throttle_in, 0, maxThrottle, 0, 1024);
    
  analogWriteResolution(Res);
  analogWrite(outOne, Speed);
  analogWriteResolution(Res);
  analogWrite(outTwo, Speed);
  //Serial.print("DAC Output: ");                            //Debug Statements ~ visual data
  //Serial.println(Speed);
}
