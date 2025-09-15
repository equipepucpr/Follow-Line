//Codigo follow line
//Equipe PUCPR
#include <QTRSensors.h>

#define Kp 0.01// experimentar para determinar isso, comece com algo pequeno que apenas faça seu bot seguir a linha em uma velocidade lenta
#define Kd 1.5 // experimente para determinar isso, aumente lentamente as velocidades e ajuste esse valor. (Nota: Kp < Kd)
#define Ki 0
#define MaxSpeed 55 // velocidade maxima do robo
#define BaseSpeed 30 // esta é a velocidade na qual os motores devem girar quando o robô estiver perfeitamente na linha
//#define NUM_SENSORS  8     // numeros de sensores usados

#define speedturn 50

#define rightMotor1 A1
#define rightMotor2 A2
#define rightMotorPWM 10
#define leftMotor1 A4
#define leftMotor2 A5
#define leftMotorPWM 11


QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

//QTRSensorsRC qtrrc((unsigned char[]) {2,3,4,5,6,7,8,9} ,NUM_SENSORS, 2500, QTR_NO_EMITTER_PIN);

//unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 3, 4, 5, 6, 7, 8, 9}, SensorCount);
  qtr.setEmitterPin(12);
  
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  delay(3000);
    Serial.begin(9600);
    Serial.println();

 // sensor_calibrate();

int i;
  for (int i = 0; i < 100; i++) // calibrar por algum tempo deslizando os sensores pela linha, ou você pode usar a calibração automática em vez disso
  {
  
   //comente esta parte para calibração automática 
    if ( i  < 25 || i >= 75 ) // vira para a esquerda e para a direita para expor os sensores às leituras mais claras e mais escuras que podem ser encontradas
    {
      move(1, 70, 1);//motor 
      move(0, 70, 0);//motor  
    }
    else
    {
      move(1, 70, 0);//motor 
      move(0, 70, 1);//motor   
    }
    qtr.calibrate();   
    delay(20);
  
  }
  
  wait();
  delay(3000); // espere 3s para posicionar o bot antes de entrar no loop principal

}  

int lastError = 0;
//unsigned int sensors[8];
uint16_t position = qtr.readLineBlack(sensorValues);

  
void loop()
{  
  position = qtr.readLineBlack(sensorValues); 
  // obter leituras calibradas junto com a posição da linha, consulte a Biblioteca do Arduino de Sensores QTR para obter mais detalhes sobre a posição da linha.
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  
 if(position>6500){
    move(1, speedturn, 1);//motor 
    move(0, speedturn, 0);//motor 
    return;
  }
  if(position<500){ 
    move(1, speedturn, 0);//motor 
    move(0, speedturn, 1);//motor 
    return;
  } 

  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > MaxSpeed ) {rightMotorSpeed = MaxSpeed;} // evitar que o motor ultrapasse a velocidade máxima
  if (leftMotorSpeed > MaxSpeed ) {leftMotorSpeed = MaxSpeed;} // evitar que o motor ultrapasse a velocidade máxima
  if (rightMotorSpeed < 0)  {rightMotorSpeed = 0;}    
  if (leftMotorSpeed < 0) {leftMotorSpeed = 0;}
    
  move(1, rightMotorSpeed, 1);//motor 
  move(0, leftMotorSpeed, 1);//motor 
  
}
  
void wait(){

  analogWrite(leftMotorPWM, 0);
  analogWrite(rightMotorPWM, 0);
  
}

void move(int motor, int speed, int direction){
  
  boolean inPin1;
  boolean inPin2;
  
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

void sensor_calibrate()
{

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // desliga o LED do Arduino para indicar que terminamos com a calibração

  //Printa os valores mínimos de calibração medidos quando os emissores estavam ligados
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  //Printa os valores maximos de calibração medidos quando os emissores estavam ligados
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);  
  
  
}
