int sensor1 = A0;
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;
int sensor5 = A4;
int rmf = 3;
int rmb = 5;
int lmf = 6;
int lmb = 9;
int MaxSP = 255;
float number[5];
int i;
int dt = 1000;

void setup() {
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
}

void loop() {
  number[0] = (5./1023.)*analogRead(sensor1);
  number[1] = (5./1023.)*analogRead(sensor2);
  number[2] = (5./1023.)*analogRead(sensor3);
  number[3] = (5./1023.)*analogRead(sensor4);
  number[4] = (5./1023.)*analogRead(sensor5);
  if(number[0]<3.5 && number[1]<3.5 && number[2]>=3.5 && number[3]<3.5 && number[4]<3.5){
    analogWrite(lmf, MaxSP);
    analogWrite(lmb, 0);
    analogWrite(rmf, MaxSP);
    analogWrite(rmb, 0);
  }
  if(number[0]<3.5 && number[1]>=3.5 && number[2]<3.5 && number[3]<3.5 && number[4]<3.5){
    analogWrite(lmf, MaxSP/2);
    analogWrite(lmb, 0);
    analogWrite(rmf, MaxSP);
    analogWrite(rmb, 0);
  }
  /*if(number[0]>=3.5 && number[1]>=3.5 && number[2]<3.5 && number[3]<3.5 && number[4]<3.5){
    analogWrite(lmf, 0);
    analogWrite(lmb, 0);
    analogWrite(rmf, MaxSP);
    analogWrite(rmb, 0);
  }*/
  if(number[0]>=3.5 && number[1]<3.5 && number[2]<3.5 && number[3]<3.5 && number[4]<3.5){
    analogWrite(lmf, 0);
    analogWrite(lmb, 0);
    analogWrite(rmf, MaxSP);
    analogWrite(rmb, 0);
  }
  if(number[0]<3.5 && number[1]<3.5 && number[2]<3.5 && number[3]>=3.5 && number[4]<3.5){
    analogWrite(lmf, MaxSP);
    analogWrite(lmb, 0);
    analogWrite(rmf, MaxSP/2);
    analogWrite(rmb, 0);
  }
  /*if(number[0]<3.5 && number[1]<3.5 && number[2]<3.5 && number[3]>=3.5 && number[4]>=3.5){
    analogWrite(lmf, MaxSP);
    analogWrite(lmb, 0);
    analogWrite(rmf, 0);
    analogWrite(rmb, 0);
  }*/
  if(number[0]<3.5 && number[1]<3.5 && number[2]<3.5 && number[3]<3.5 && number[4]>=3.5){
    analogWrite(lmf, MaxSP);
    analogWrite(lmb, 0);
    analogWrite(rmf, 0);
    analogWrite(rmb, 0);
  }
}
