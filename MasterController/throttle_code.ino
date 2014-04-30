int steering = A10;
int throttle = A11;
int outOne = DAC0;
int outTwo = DAC1;
int Res = 10;  
int Speed = 0;
int maxThrottle = 450;    // NEEDS TO BE MEASURED


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly: 
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
