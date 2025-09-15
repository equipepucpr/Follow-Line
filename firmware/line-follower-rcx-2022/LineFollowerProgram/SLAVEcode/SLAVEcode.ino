#include <SoftwareSerial.h>
#include <Wire.h>  // i2C Conection Library
#include <LiquidCrystal_I2C.h>  //Memanggil i2C LCD Library

//LCDi2C
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3,POSITIVE);

SoftwareSerial mySerial(10, 11); // RX, TX pada arduino

float interval_elapsed, interval_limit;

int state = 20;
unsigned long t;

void setup() {
    Serial.begin(38400);
    mySerial.begin(38400);
    interval_elapsed=0;
    interval_limit=20;
    Serial.println("CLEARDATA");
    Serial.println("LABEL,Date,Time,Millis,RandonValue");

}

void loop(){
  if(mySerial.available()>0) {
    state = mySerial.read();   
  }
      int data = map(state, 0, 255, 0, 7000);
      if (state > 7000){
        state == 7000;
      }
      else if (state <= 0){
        state == 0;
      }
      else{
        state == state;
      }
    t = millis();
    interval_elapsed = interval_elapsed + 0.01;
  
    if (interval_elapsed >= interval_limit)
    {

//        Serial.println( (String) "DATA,DATE,TIME," + t + "," + data )   ;   
//        delay(50);
      Serial.print(data);
      Serial.print(" ");
      Serial.print("3500");
      Serial.print(" ");
      Serial.print("0");
      Serial.print(" ");
      Serial.println("7000");
      interval_elapsed=0;
    }
    else {interval_elapsed=interval_elapsed;}

}
