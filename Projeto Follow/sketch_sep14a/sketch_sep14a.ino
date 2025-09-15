// definir botão
int botao1 = 2;
int botao2 = 3;
int botao3 = 4;

// definir led
int led1 = 6;
int led2 = 7;

//Variavel que contem o estado do botao(0 LOW, 1 HIGH)
int estadoBotao1 = 0;
int estadoBotao2 = 0;
int estadoBotao3 = 0;

void setup(){
  pinMode(botao1, INPUT); //ENTRADA
  pinMode(botao2, INPUT); //ENTRADA
  pinMode(botao3, INPUT); //ENTRADA
  pinMode(led1, OUTPUT); //SAIDA
  pinMode(led1, OUTPUT); //SAIDA
}

void loop(){
  estadoBotao1 = digitalRead(botao1);
  estadoBotao2 = digitalRead(botao2);
  estadoBotao3 = digitalRead(botao3);
  
  // Primeira função led1 ||
  if(estadoBotao1 == HIGH || estadoBotao2 == HIGH){
    digitalWrite(led1,HIGH);
  }
  if(estadoBotao1 == HIGH || estadoBotao2 == LOW){
    digitalWrite(led1,LOW);
  }  
  if(estadoBotao1 == LOW || estadoBotao2 == HIGH){
    digitalWrite(led1,HIGH);
  }
}
