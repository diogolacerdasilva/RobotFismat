int sensor = A0;
int sensor1 = A1;
int sensor2 = A2;
int sensor3 = A3;
int sensor4 = A4;
float number[5];
int i;
int dt = 1000;

void setup() {
  pinMode(sensor, INPUT);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  Serial.begin(9600);
}

void loop() {
  number[0] = analogRead(sensor);
  Serial.print("Valor do Sensor 1 : ");
  Serial.println((5./1023.)*number[0]);
  number[1] = analogRead(sensor1);
  Serial.print("Valor do Sensor 2 : ");
  Serial.println((5./1023.)*number[1]);
  number[2] = analogRead(sensor2);
  Serial.print("Valor do Sensor 3 : ");
  Serial.println((5./1023.)*number[2]);
  number[3] = analogRead(sensor3);
  Serial.print("Valor do Sensor 4 : ");
  Serial.println((5./1023.)*number[3]);
   number[4] = analogRead(sensor4);
  Serial.print("Valor do Sensor 5 : ");
  Serial.println((5./1023.)*number[4]);
  Serial.println(" ");
  delay(dt);
}