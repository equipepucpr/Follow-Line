//Driver MotorDC
//Motor A
#define rightMotor1 5 //pin driver1
#define rightMotor2 4 //pin driver2
#define rightMotorPWM 9 //pin pwmA

//Motor B 
#define leftMotor1 7 //pin driver3
#define leftMotor2 8 //pin driver4
#define leftMotorPWM 3 //pin pwmB

#define MaxSpeed 255// max speed of the robot
#define BaseSpeed 255 // this is the speed at which the motors should spin when the robot is perfectly on the line

int pot1 = A0;
int SetSpeed = 0;

void setup() {
  Serial.begin(9600);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  SetSpeed = 250;

}

void loop() {
//    SetSpeed = analogRead(pot1);
//    SetSpeed = map(SetSpeed, 0, 1023, 0, 255);

    motorKiri();
    delay(1500);
    motorKanan();
    delay(3000);
    motorKiri();
    delay(1500);    
    motorMaju();
    delay(2000);
    motorMundur();
    delay(2000);
    motorStop();
    delay(2000);
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

void motorStop(){
    digitalWrite(rightMotor1,LOW);
    digitalWrite(rightMotor2,LOW);
    analogWrite(rightMotorPWM,0);
    digitalWrite(leftMotor1,LOW);
    digitalWrite(leftMotor2,LOW);
    analogWrite(leftMotorPWM,0);
}

void motorKanan(){
    digitalWrite(rightMotor1,HIGH);
    digitalWrite(rightMotor2,LOW);
    analogWrite(rightMotorPWM,70);
    digitalWrite(leftMotor1,LOW);
    digitalWrite(leftMotor2,HIGH);
    analogWrite(leftMotorPWM,70); 
}

void motorKiri(){
    digitalWrite(rightMotor1,LOW);
    digitalWrite(rightMotor2,HIGH);
    analogWrite(rightMotorPWM,70);
    digitalWrite(leftMotor1,HIGH);
    digitalWrite(leftMotor2,LOW);
    analogWrite(leftMotorPWM,70); 
}

void motorMaju(){
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, SetSpeed);
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, SetSpeed);
}

void motorMundur(){
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    analogWrite(rightMotorPWM, SetSpeed);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    analogWrite(leftMotorPWM, SetSpeed);
}
