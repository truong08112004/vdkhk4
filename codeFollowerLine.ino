#include <Arduino.h>

//--------Pin definitions for the L298N Motor Driver----
#define AIN1 7
#define AIN2 6
#define BIN1 5
#define BIN2 4
#define PWMA 3
#define PWMB 8

#define A1 A1
#define A2 A2
#define A3 A3
#define A4 A4
#define A5 A5
int S[7] = {0, A1, A2, A3, A4, A5, 0};
//------------------------------------------------------------

//--------Enter Line Details here---------
bool isBlackLine = true;            // keep true in case of black line. In case of white line change this to false
unsigned int lineThickness = 15;    // Enter line thickness in mm. Works best for thickness between 10 & 35
unsigned int numSensors = 5;        // Enter number of sensors as 5 or 7
//-----------------------------------------

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 255;
int currentSpeed = 30;
int sensorWeight[7] = { 4, 2, 1, 0, -1, -2, -4 };
int activeSensors;
float Kp = 0.08;
float Kd = 0.15;
float Ki = 0;

int onLine = 1;
int minValues[7], maxValues[7], threshold[7], sensorValue[7], sensorArray[7];
bool isStart = true;
void setup() {
  Serial.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  analogWrite(PWMA, 255);
  analogWrite(PWMB, 255);

  lineThickness = constrain(lineThickness, 10, 35);
  if (numSensors == 5) {
    sensorWeight[1] = 4;
    sensorWeight[5] = -4;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  calibrate();
  while (isStart) {
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {  // PID LINE FOLLOW
      linefollow();
    } else {
      if (error > 0) {
        motor1run(-200);
        motor2run(lfSpeed);
      } else {
        motor1run(lfSpeed);
        motor2run(-200);
      }
    }
    if (activeSensors == 5) {
      while(true){
          motor1run(0);
  motor2run(0);
        }
      isStart = false;
    }

  }
}

void linefollow() {
  error = 0;
  activeSensors = 0;

  if (numSensors == 7) {
    for (int i = 0; i < 7; i++) {
      error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
      activeSensors += sensorArray[i];
    }
    error = error / activeSensors;
  }
  if (numSensors == 5) {
    for (int i = 1; i < 6; i++) {
      error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
      activeSensors += sensorArray[i];
    }
    error = error / activeSensors;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;


  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = -230;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = -230;
  }
  motor1run(lsp);
  motor2run(rsp);
}

void calibrate() {
  for (int i = 0; i < 7; i++) {
    minValues[i] = 25;
    maxValues[i] = 475;
  }

  for (int i = 0; i < 7; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void readLine() {
  onLine = 0;
  if (numSensors == 7) {
    for (int i = 0; i < 7; i++) {
      int val = analogRead(i);
      if (isBlackLine) {
        sensorValue[i] = map(val, minValues[i], maxValues[i], 0, 1000);
      } else {
        sensorValue[i] = map(val, minValues[i], maxValues[i], 1000, 0);
      }
      sensorValue[i] = constrain(sensorValue[i], 0, 1000);
      sensorArray[i] = sensorValue[i] > 500;
      if (sensorArray[i]) onLine = 1;

      if (isBlackLine == true && sensorArray[i]) onLine = 1;
      if (isBlackLine == false && !sensorValue[i]) onLine = 1;
    }
  }
  if (numSensors == 5) {
    for (int i = 1; i < 6; i++) {
      int val = analogRead(S[i]);
      if (isBlackLine) {
        sensorValue[i] = map(val, minValues[i], maxValues[i], 0, 1000);
      } else {
        sensorValue[i] = map(val, minValues[i], maxValues[i], 1000, 0);
      }
      sensorValue[i] = constrain(sensorValue[i], 0, 1000);
      sensorArray[i] = sensorValue[i] > 500;
      if (sensorArray[i]) onLine = 1;

      if (isBlackLine == true && sensorArray[i]) onLine = 1;
      if (isBlackLine == false && !sensorValue[i]) onLine = 1;
    }
  }
}

void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}
