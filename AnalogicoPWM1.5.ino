int sensor[] = {A0,A1,A2,A3,A4};
int rmf = 3;
int rmb = 5;
int lmf = 6;
int lmb = 9;
int MaxSP = 255;
int number[5];
int i;
int dt = 1000;

void setup() {
  for(i=0;i<5;i++)
  {
    pinMode(sensor[i], INPUT);
  }
}

void loop() {
  for(i=0;i<5;i++)
  {
    number[i] = (5./1023.)*analogRead(sensor[i]); 
  }
  
  if(number[0]>=3.5){
    analogWrite(lmf, 0);
    analogWrite(lmb, 0);
    analogWrite(rmf, MaxSP);
    analogWrite(rmb, 0);
  }
  else if(number[4]>=3.5){
    analogWrite(lmf, MaxSP);
    analogWrite(lmb, 0);
    analogWrite(rmf, 0);
    analogWrite(rmb, 0);
  }
  else if(number[1]>=3.5){
    analogWrite(lmf, MaxSP/2);
    analogWrite(lmb, 0);
    analogWrite(rmf, MaxSP);
    analogWrite(rmb, 0);
  }
  else if(number[3]>=3.5){
    analogWrite(lmf, MaxSP);
    analogWrite(lmb, 0);
    analogWrite(rmf, MaxSP/2);
    analogWrite(rmb, 0);
  }
  else if(number[2]>=3.5){
    analogWrite(lmf, MaxSP);
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
  /*if(number[0]<3.5 && number[1]<3.5 && number[2]<3.5 && number[3]>=3.5 && number[4]>=3.5){
    analogWrite(lmf, MaxSP);
    analogWrite(lmb, 0);
    analogWrite(rmf, 0);
    analogWrite(rmb, 0);
  }*/
}
