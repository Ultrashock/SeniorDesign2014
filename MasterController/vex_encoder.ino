/* This Code will read the values from the Vex Encoders and will compute an RPM 
   Version 1.0 - Created by: Jennifer Cory, Brandon Zschokke, and Hallie Sheaffer
     Created vex encoder code that will take readings and convert to RPM
   Version 1.1- Created by: Brandon Zschokke
     Duplicated vex encoder code to include both encoders
*/
   

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


 void setup() { 
   pinMode (encoder0PinA,INPUT);
   pinMode (encoder0PinB,INPUT);
   pinMode (encoder1PinA,INPUT);
   pinMode (encoder1PinB,INPUT);
   Serial.begin (9600);
 } 

 void loop() { 
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
  // Serial.print("RPM: ");
  // Serial.print(RPM);
  // Serial.println();
 encoder0Pos=0;
 encoder1Pos=0;
 }
