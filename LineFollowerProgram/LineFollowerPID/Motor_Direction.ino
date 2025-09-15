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
