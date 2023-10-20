#define sens_4 7 // sensor de linha 4
#define sens_2 8 // sensor de linha 2
#define m_inv1 9 // bit de inversão do motor 1
#define m_pwm1 6 // bit de controle pwm do motor 1 
#define m_inv2 5 // bit de inversão do motor 2
#define m_pwm2 3 // bit de controle pwm do motor 2

int speed = 100;
int speed2 = 255; // Velocidade máxima dos motores
int lastSensor = 0; // Variável para rastrear o último sensor que detectou uma linha

void setup() {
  pinMode(sens_2, INPUT);
  pinMode(sens_4, INPUT);
  pinMode(m_inv1, OUTPUT);
  pinMode(m_pwm1, OUTPUT);
  pinMode(m_inv2, OUTPUT);
  pinMode(m_pwm2, OUTPUT);

  digitalWrite(m_inv1, LOW); // Robô move-se para frente
  digitalWrite(m_inv2, LOW);
  analogWrite(m_pwm1, 0); // motor 1 inicia parado
  analogWrite(m_pwm2, 0); // motor 2 inicia parado
}
void loop() {
  bool sensor2Active = !digitalRead(sens_2);
  bool sensor4Active = !digitalRead(sens_4);

  if (sensor2Active && sensor4Active) {
    // Ambos os sensores detectam a linha: andar para frente
    analogWrite(m_pwm1, speed);
    analogWrite(m_pwm2, speed);
  } else if (sensor2Active) {
    lastSensor = 2; // Define o último sensor como 2
    analogWrite(m_pwm1, speed2);
    analogWrite(m_pwm2, 0);
  } else if (sensor4Active) {
    lastSensor = 4; // Define o último sensor como 4
    analogWrite(m_pwm1, 0);
    analogWrite(m_pwm2, speed2);
  } 
}