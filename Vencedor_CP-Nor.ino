#define DEBUG
#define BT_NAME "Teralf"

#ifdef DEBUG
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif
BluetoothSerial SerialBT;  // Bluetooth Serial instance
#endif

#include <RoboCore_Vespa.h>  // Library for the Vespa microcontroller
#include <QTRSensors.h>      // Library for the QTR-8A or the QTR-8RC

VespaMotors motor;  // Vespa Motor Object
QTRSensors qtr;     // QTR Sensor
QTRSensors qtra;    // QTR Analog Sensor

// Set button and led pins
const uint8_t PIN_BUTTON = 35;
const uint8_t PIN_LED = 15;

// Setup of the module of sensors
const uint8_t SENSOR_COUNT = 8;       // The number of sensors, which should match the length of the pins array
uint16_t sensorValues1[SENSOR_COUNT];  // An array in which to store the calibrated sensor readings

const uint8_t SENSOR_COUNT_A = 2;
uint16_t sensorValues2[SENSOR_COUNT_A];  // An array in which to store the calibrated sensor readings

// Maximum line position, considering the amount of sensors.
const long MAX_POSITION = (SENSOR_COUNT - 1) * 1000;

// Limit value of the margin of error
uint8_t marginError = 5;
uint8_t marginErrorStd = 5;
uint8_t marginErrorAdd = 0;

// Mark counting
const uint8_t numeros_true[] = {0, 2, 4, 7, 9, 11, 13, 15, 18, 20, 25, 27, 29, 33, 36, 38, 42, 44, 46, 49, 51, 53, 55, 57, 65, 68, 74, 76};
const uint8_t tamanho_array = sizeof(numeros_true) / sizeof(numeros_true[0]);
uint8_t line_check = 0;
uint8_t line_check_dir = 0;
const uint8_t line_total_dir = 2;
bool sensorDetector = false;
bool sensorDetector_dir = false;

// Bluetooth
bool firstRun = true;

// PID Control
float p = 0, i = 0, d = 0, pid = 0, error = 0, lastError = 0;
float Kp = 8.343;
float Ki = 0.0;
float Kd = 212.55;

float maxIntegral = 100000.0;  // Anti-windup limit for the integral term
float dFilterCoeff = 0.1;    // Coefficient for the low-pass filter

int maxSpeed = 85;
int maxSpeed_Std = 85;
int maxSpeed_Low = 85;
int lSpeed, rSpeed;

const bool LINE_BLACK = false;

void detectMarker(uint16_t markerValue, bool &detector, uint8_t &line_check, bool is_dir = false) {
  uint16_t threshold = is_dir ? 3000 : 1000;
  if (markerValue < threshold && !detector) {
    line_check++;
    detector = true;
  } else if (markerValue >= threshold) {
    detector = false;
  }
}


void setup() {
  qtr.setTypeRC();  // For QTR-8RC Sensor pins:
  qtr.setSensorPins((const uint8_t[]){ 16, 5, 19, 21, 22, 23, 18, 17 }, SENSOR_COUNT);
  qtra.setTypeAnalog();  // For QTR-8A Sensor pins:
  qtra.setSensorPins((const uint8_t[]){ 36, 39 }, SENSOR_COUNT_A);

  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_LED, OUTPUT);

#ifdef DEBUG
  if (firstRun) {
    Serial.begin(115200);
    delay(100);
    SerialBT.begin(BT_NAME);  // Bluetooth device name
    firstRun = false;
  }

  SerialBT.println("Start BT communication");

  String btMessage;
  String prefix;

  while (prefix != "end" && digitalRead(PIN_BUTTON) == HIGH) {
    btMessage = receiveBtMessage();
    prefix = getPrefix(btMessage);

    if (prefix == "pid") {
      Kp = getNumber(btMessage, 1);
      Ki = getNumber(btMessage, 2);
      Kd = getNumber(btMessage, 3);
    } else if (prefix == "spe") {
      maxSpeed_Std = getNumber(btMessage, 1);
      maxSpeed_Low = getNumber(btMessage, 2);
    } else if (prefix == "err") {
      marginErrorStd = getNumber(btMessage, 1);
    } else if (prefix == "pri") {
      printParameters();
    } else if (prefix == "end") {
      break;
    } else {
      SerialBT.println("This command doesn't exist!");
    }
  }

  printParameters();
  SerialBT.println("Start Calibration...");
  delay(500);
#endif

  // Calibration
  digitalWrite(PIN_LED, HIGH);
  while (digitalRead(PIN_BUTTON) == HIGH) {  // Calibrates until the button is pressed
    qtr.calibrate();
  }
  digitalWrite(PIN_LED, LOW);

#ifdef DEBUG
  // Print the calibration minimum and maximum values measured when emitters were on
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    SerialBT.print(qtr.calibrationOn.minimum[i]);
    SerialBT.print(' ');
  }
  SerialBT.println();
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    SerialBT.print(qtr.calibrationOn.maximum[i]);
    SerialBT.print(' ');
  }
  SerialBT.println();
#endif

  delay(2000);  // Start loop after 2 seconds
}

void loop() {
  qtr.read(sensorValues1);
  qtra.read(sensorValues2);

  error = map(qtr.readLineWhite(sensorValues1), 0, MAX_POSITION, -1000, 1000);

  // Marker detection
  detectMarker(sensorValues2[1], sensorDetector_dir, line_check_dir, true);

  if (line_check_dir >= line_total_dir && sensorValues2[1] < 4000) {
    maxSpeed_Low = maxSpeed;
    for(int decrease = 0; decrease < maxSpeed_Low; decrease++){
      maxSpeed=maxSpeed - 1;
    }
  }

  /* Adjust speed and margin error
  adjustSpeedAndMargin(line_check);*/

  // Calculate PID
  calculatePID(error);

  // Control Motors
  controlMotors();

  Serial.print(sensorValues2[1]);
  Serial.print(",");
  Serial.print(line_check_dir);
  Serial.print(",");
  Serial.print(rSpeed);
  Serial.print(",");
  Serial.print(Kp * p);
  Serial.print(",");
  Serial.print(Ki * i);
  Serial.print(",");
  Serial.print(Kd * d);
  Serial.print(",");
  Serial.println(pid / maxSpeed);
}

/*void adjustSpeedAndMargin(uint8_t line_check) {
  if (verificarNumero(line_check)) {
    marginError = marginErrorAdd + marginErrorStd;
    maxSpeed = maxSpeed_Std;
  } else {
    marginError = marginErrorStd;
    maxSpeed = maxSpeed_Low;
  }
}*/

void calculatePID(float error) {
  p = error;
  i += error;

  // Anti-windup: Limit the integral term
  if (i > maxIntegral) i = maxIntegral;
  else if (i < -maxIntegral) i = -maxIntegral;

  // Filtered derivative term
  float rawD = error - lastError;
  d = (dFilterCoeff * rawD) + ((1 - dFilterCoeff) * d);

  pid = (Kp * p) + (Ki * i) + (Kd * d);
  lastError = error;
}

void controlMotors() {
  lSpeed = maxSpeed - pid / maxSpeed;
  rSpeed = maxSpeed + pid / maxSpeed;

  lSpeed = constrain(lSpeed, -maxSpeed, maxSpeed);
  rSpeed = constrain(rSpeed, -maxSpeed, maxSpeed);

  if (error >= -marginError && error <= marginError) {
    i = 0;
    motor.turn(maxSpeed, maxSpeed);
  } else {
    motor.turn(lSpeed, rSpeed);
  }
}

#ifdef DEBUG
String receiveBtMessage() {
  String message;
  char incomingChar;

  digitalWrite(PIN_LED, HIGH);
  while (digitalRead(PIN_BUTTON) == HIGH) {
    if (SerialBT.available()) {
      incomingChar = SerialBT.read();
      if (incomingChar == '\n') break;
      message += String(incomingChar);
    }
  }
  digitalWrite(PIN_LED, LOW);

  message.trim();
  return message;
}

String getPrefix(String data) {
  return getElement(data, 0);
}

double getNumber(String data, int index) {
  return atof(getElement(data, index).c_str());
}

String getElement(String data, int index) {
  char separator = ' ';
  int found = 0;
  int startIndex = 0, endIndex = -1;
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      startIndex = endIndex + 1;
      endIndex = (i == maxIndex) ? i + 1 : i;
    }
  }

  if (found <= index) {
    return "";
  }

  return data.substring(startIndex, endIndex);
}

void printParameters() {
  SerialBT.println("Configured parameters:");
  SerialBT.print(">> P: ");
  SerialBT.print(Kp, 4);
  SerialBT.print(" | I: ");
  SerialBT.print(Ki, 4);
  SerialBT.print(" | Kd: ");
  SerialBT.println(Kd, 4);

  SerialBT.print(">> Speed: ");
  SerialBT.print(maxSpeed_Std);
  SerialBT.print(", ");
  SerialBT.println(maxSpeed_Low);


  SerialBT.print(">> Margin Error: ");
  SerialBT.println(marginError);
}

#endif

bool verificarNumero(int numero) {
  for (int i = 0; i < tamanho_array; i++) {
    if (numero == numeros_true[i]) {
      return true;
    }
  }
  return false;
}

