#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

// Motor pins
#define PWMRIGHT 9
#define RIGHTIN2 8
#define RIGHTIN1 7

#define LEFTIN1 5
#define LEFTIN2 4
#define PWMLEFT 3

#define LED 13

// -------- PD PARAMETERS --------
float Kp = 0.23;
float Kd = 2.0;
int baseSpeed = 200;

// -------- LOST LINE PARAMETERS --------
#define BLACK_THRESHOLD 800
#define RECOVERY_SPEED 120

int lastError = 0;

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins(
    (const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7},
    SensorCount
  );

  pinMode(RIGHTIN2, OUTPUT);
  pinMode(RIGHTIN1, OUTPUT);
  pinMode(PWMRIGHT, OUTPUT);
  pinMode(LEFTIN1, OUTPUT);
  pinMode(LEFTIN2, OUTPUT);
  pinMode(PWMLEFT, OUTPUT);
  pinMode(LED, OUTPUT);

  delay(2000);

  digitalWrite(LED, HIGH);
  for (int i = 0; i < 300; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED, LOW);
}

void loop() {
  // Read sensors
  unsigned int linePosition = qtr.readLineBlack(sensorValues);

  // -------- LOST LINE DETECTION --------
  bool lineDetected = false;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > BLACK_THRESHOLD) {
      lineDetected = true;
      break;
    }
  }

  if (!lineDetected) {
    lostLineRecovery();
    return;
  }
  // ------------------------------------

  // Error calculation (center = 3500)
  int error = 3500 - linePosition;

  // -------- PD CONTROL --------
  int derivative = error - lastError;
  int adjustment = (Kp * error) + (Kd * derivative);
  lastError = error;
  // ----------------------------

  int leftMotorPwmValue  = baseSpeed + adjustment;
  int rightMotorPwmValue = baseSpeed - adjustment;

  leftMotorPwmValue  = constrain(leftMotorPwmValue, 0, 170);
  rightMotorPwmValue = constrain(rightMotorPwmValue, 0, 170);

  leftMotorForward(leftMotorPwmValue);
  rightMotorForward(rightMotorPwmValue);
}

// -------- MOTOR FUNCTIONS --------
void leftMotorForward(int speed) {
  digitalWrite(LEFTIN1, LOW);
  digitalWrite(LEFTIN2, HIGH);
  analogWrite(PWMLEFT, speed);
}

void rightMotorForward(int speed) {
  digitalWrite(RIGHTIN1, HIGH);
  digitalWrite(RIGHTIN2, LOW);
  analogWrite(PWMRIGHT, speed);
}

// -------- LOST LINE RECOVERY --------
void lostLineRecovery() {
  if (lastError > 0) {
    // Turn LEFT
    digitalWrite(LEFTIN1, LOW);
    digitalWrite(LEFTIN2, HIGH);
    analogWrite(PWMLEFT, RECOVERY_SPEED);

    analogWrite(PWMRIGHT, 0);
  } else {
    // Turn RIGHT
    digitalWrite(RIGHTIN1, HIGH);
    digitalWrite(RIGHTIN2, LOW);
    analogWrite(PWMRIGHT, RECOVERY_SPEED);

    analogWrite(PWMLEFT, 0);
  }
}
