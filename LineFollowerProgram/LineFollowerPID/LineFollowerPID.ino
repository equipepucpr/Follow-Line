#include <QTRSensors.h>
#include <SoftwareSerial.h>

QTRSensors qtr;
SoftwareSerial mySerial(10, 11); // RX, TX pada arduino

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

//Driver MotorDC
//Motor A
#define rightMotor1 5 //pin driver1
#define rightMotor2 4 //pin driver2
#define rightMotorPWM 9 //pin pwmA

//Motor B 
#define leftMotor1 7 //pin driver3
#define leftMotor2 8 //pin driver4
#define leftMotorPWM 3 //pin pwmB

#define MaxSpeed 80
#define BaseSpeed 80

// this is the speed at which the motors should spin when the robot is perfectly on the line

//PID
float Kp,Ti,Td,sv,pv,Ki,Kd,Ts;
float et,et_1,et_2;
int ut,ut_1;

void setup() {
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(2);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);

  delay(300);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i <= 60; i++)
  {
    if (i <= 10){
      motorKiri();
    }
    else if (i >= 11 && i<=30){
      motorKanan();
    }
    else if (i >= 31 && i <= 50){
      motorKiri(); 
    }
    else if (i >= 51 && i <= 60){
      motorKanan();
    }
    qtr.calibrate();
    delay(1);
  }
  motorStop();
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(9600);
  mySerial.begin(38400);
  mySerial.print("Bluetooth Terhubung");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  sv = 3500;
  Kp = 1.2;
  Ki = 4.;
  Kd = 0.075;
  Ts = 0.01;
  et = 0;
  et_1 = 0;
  et_2 = 0;
  ut_1 = 0;
}

void loop() {

  int position = qtr.readLineBlack(sensorValues);
  
  et = position - 3500;

  ut = ut_1 + (Kp*(et-et_1)) + (Ki*et*Ts) + (Kd*((et-(2*et_1)+et_2)/Ts));
  //if (ut > 255) {ut = 255;}
  //else if (ut < 0) {ut = 0;}
  //else{ut = ut; }

  int rightMotorSpeed = BaseSpeed + ut;
  int leftMotorSpeed = BaseSpeed - ut;
  
  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  {
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, rightMotorSpeed);
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, leftMotorSpeed);
  }
   int data = map(position,0,7000,0,255);
   mySerial.write(data);
   Serial.println(data);
    
   et_1 = et;
   et_2 = et_1;
   ut_1 = ut;
}

void move(int motor, int speed, int direction){

  boolean inPin1=HIGH;
  boolean inPin2=LOW;
  
  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }  
  if(direction == 0){
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if(motor == 0){
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  }
  if(motor == 1){
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  }  
}
