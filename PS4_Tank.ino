#include <PS4Controller.h>
#include <ESP32Servo.h>
#include <vector>
//start servo
Servo Gripper;
Servo Base;
Servo Shoulder;
Servo Elbow;

int ShoulderPin = 26;
int GripperPin = 27;

int ShoulderDeg = 90;
int GripperDeg = 30;

void setupServo()
{
  Gripper.attach(GripperPin);
  Gripper.write(GripperDeg);

  Shoulder.attach(ShoulderPin);
  Shoulder.write(ShoulderDeg);
}
///end servo

int led = 13;
bool ledcontrol = false;
//Right motor
int enableRightMotor=22; 
int rightMotorPin1=16;
int rightMotorPin2=17;
//Left motor
int enableLeftMotor=23;
int leftMotorPin1=18;
int leftMotorPin2=19;

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

#define MAX_MOTOR_SPEED 255  

int rightMotorSpeed = 0;
int leftMotorSpeed = 0;
int defaultSpeed = 255;

void notify()
{
  if (PS4.Triangle())
  {
     ShoulderDeg+=1;
     if (ShoulderDeg > 170) ShoulderDeg = 170; 
     Shoulder.write(ShoulderDeg);
  }
  if (PS4.Cross())
  {
     ShoulderDeg-=1;
     if (ShoulderDeg < 40) ShoulderDeg = 40; 
     Shoulder.write(ShoulderDeg);
  }
   if (PS4.Square())
  {
     delay(500);
     ShoulderDeg+=1;
     if (ShoulderDeg > 170) ShoulderDeg = 170; 
     Shoulder.write(ShoulderDeg);
  }
  if (PS4.Circle())
  {
     delay(500);
     ShoulderDeg-=1;
     if (ShoulderDeg < 40) ShoulderDeg = 40; 
     Shoulder.write(ShoulderDeg);
  }
  if (PS4.R1())
  {
    GripperDeg+=1;
    if (GripperDeg > 70) GripperDeg = 60; 
    Gripper.write(GripperDeg); 
  }
  if (PS4.R2())
  {
    GripperDeg-=1;
     if (GripperDeg < 20) GripperDeg = 20; 
     Gripper.write(GripperDeg); 
  }


 if (PS4.Up())             //Move car Forward
  {
    rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else if (PS4.Down())      //Move car Backward
  {
    rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (PS4.Right())     //Move car Right
  {
    rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (PS4.Left())      //Move car Left
  {
    rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else                                //Stop the carì
  {
    rotateMotor(0, 0);
  } 
  
  if (PS4.Options()) {
    ledcontrol =  !ledcontrol;
  }
  
 
}

void onConnect()
{
  Serial.println("Connected!.");
}

void onDisConnect()
{
  rotateMotor(0, 0);
  Serial.println("Disconnected!.");    
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }
  
  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  } 
  ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
  ledcWrite(leftMotorPWMSpeedChannel, abs(leftMotorSpeed));   
}

void setUpPinModes()
{
  pinMode(led,OUTPUT);
  
  pinMode(enableRightMotor,OUTPUT);
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);
  
  pinMode(enableLeftMotor,OUTPUT);
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);

  //Set up PWM for motor speed
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);  
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);  
  
  rotateMotor(0, 0);
}


void setup()
{
  setupServo();
  setUpPinModes();
  Serial.begin(115200);
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin();
  Serial.println("Ready.");
}

void loop()
{
    if (ledcontrol) {
    digitalWrite(led, HIGH); // Bật LED khi nút option được nhấn
  } else {
    digitalWrite(led, LOW); // Tắt LED khi nút option không được nhấn
  }
}
