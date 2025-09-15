#define enA 10//Enable1 L298 Pin enA 
#define in1 9 //Motor1  L298 Pin in1 
#define in2 8 //Motor1  L298 Pin in1 
#define in3 7 //Motor2  L298 Pin in1 
#define in4 6 //Motor2  L298 Pin in1 
#define enB 5 //Enable2 L298 Pin enB 

#define RIGHT_SENSOR_1 A0 
#define RIGHT_SENSOR_2 A1 
#define LEFT_SENSOR_1 A3
#define LEFT_SENSOR_2 A4 

void setup(){ // put your setup code here, to run once

    pinMode(RIGHT_SENSOR_1, INPUT);
    pinMode(RIGHT_SENSOR_2, INPUT);
    pinMode(LEFT_SENSOR_1, INPUT);
    pinMode(LEFT_SENSOR_2, INPUT);
    
    pinMode(enA, OUTPUT); // declare as output for L298 Pin enA 
    pinMode(in1, OUTPUT); // declare as output for L298 Pin in1 
    pinMode(in2, OUTPUT); // declare as output for L298 Pin in2 
    pinMode(in3, OUTPUT); // declare as output for L298 Pin in3   
    pinMode(in4, OUTPUT); // declare as output for L298 Pin in4 
    pinMode(enB, OUTPUT); // declare as output for L298 Pin enB 
    
    analogWrite(enA, 150); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
    analogWrite(enB, 150); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed 
    delay(1000);
}
void loop(){  
    if((digitalRead(RIGHT_SENSOR_1) == 0)&&(digitalRead(LEFT_SENSOR_1) == 0)){
        forword();
      }   //if Right Sensor and Left Sensor are at White color then it will call forword function
    else{Stop();}

    if((digitalRead(RIGHT_SENSOR_1) == 1)&&(digitalRead(LEFT_SENSOR_1) == 0)){
        turnRight();
      }   //if Right Sensor and Left Sensor are at White color then it will call forword function
    else{Stop();}

    if((digitalRead(RIGHT_SENSOR_1) == 0)&&(digitalRead(LEFT_SENSOR_1) == 1)){
        turnLeft();
      }   //if Right Sensor and Left Sensor are at White color then it will call forword function
    else{Stop();}
}

void forword(){  //forword
digitalWrite(in1, HIGH); //Right Motor forword Pin 
digitalWrite(in2, LOW);  //Right Motor backword Pin 
digitalWrite(in3, LOW);  //Left Motor backword Pin 
digitalWrite(in4, HIGH); //Left Motor forword Pin 
}

void turnRight(){ //turnRight
digitalWrite(in1, LOW);  //Right Motor forword Pin 
digitalWrite(in2, HIGH); //Right Motor backword Pin  
digitalWrite(in3, LOW);  //Left Motor backword Pin 
digitalWrite(in4, HIGH); //Left Motor forword Pin 
}

void turnLeft(){ //turnLeft
digitalWrite(in1, HIGH); //Right Motor forword Pin 
digitalWrite(in2, LOW);  //Right Motor backword Pin 
digitalWrite(in3, HIGH); //Left Motor backword Pin 
digitalWrite(in4, LOW);  //Left Motor forword Pin 
}

void Stop(){ //stop
digitalWrite(in1, LOW); //Right Motor forword Pin 
digitalWrite(in2, LOW); //Right Motor backword Pin 
digitalWrite(in3, LOW); //Left Motor backword Pin 
digitalWrite(in4, LOW); //Left Motor forword Pin 
}
