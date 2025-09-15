/*
  Sample code to test the line sensor in the LF-2 robot
*/

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue0 = analogRead(A0);
  int sensorValue1 = analogRead(A1);
  int sensorValue2 = analogRead(A2);
  int sensorValue3 = analogRead(A3);
  int sensorValue4 = analogRead(A4);
  int sensorValue5 = analogRead(A5);
  int sensorValue6 = analogRead(A6);
  int sensorValue7 = analogRead(A7);


  // print out the value you read:
  
  Serial.print(sensorValue0);
  Serial.print("   ");
  Serial.print(sensorValue1);
  Serial.print("   ");
  Serial.print(sensorValue2);
  Serial.print("   ");
  Serial.print(sensorValue3);
  Serial.print("   ");
  Serial.print(sensorValue4);
  Serial.print("   ");
  Serial.println(sensorValue5);
  Serial.print("   ");
  Serial.println(sensorValue6);
  Serial.print("   ");
  Serial.println(sensorValue7);
  Serial.print("   ");
 
  delay(1000);
}