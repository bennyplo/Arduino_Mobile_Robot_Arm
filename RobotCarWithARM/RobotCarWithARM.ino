///////////////////////////////////////////////////////////////
// RobotCarWithARM
//
// A mobile robot with a tank chassis and a 6 degree of freedom 
// robot arm. The whole system is driven by a Arduino Uno.
// The Arduino uses teh PCA9685 to connect and control all the 6 
// motors of the robotic arm, and it uses the L298N motor driver to
// drive the motors of the tank. In addition, it can be connected wirelessly
// to a computer (serial port) via bluetooth (by using HC-06 module).
//
// by Benny Lo
// July 15 2019
///////////////////////////////////////////////////////////////
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>//for controlling the robot arm
#include <SoftwareSerial.h>//for blueooth

//Robot arm - instruct the PCA9685 to control the motors
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
//robot arm motor IDs
#define CRAW_SERVO 5
#define WRIST_SERVO 4
#define ELBOW_SERVO 3
#define SHOULDER_SERVO 2
#define WAIST_SERVO 1
#define HIP_SERVO 0
//robot tank motor control I/O ports
#define RIGHT_BACKWARD 0x3
#define RIGHT_FORWARD 0x2
#define LEFT_FORWARD 0x4
#define LEFT_BACKWARD 0x5

///Bluetooth - > serial connection
char val;
SoftwareSerial mySerial(8, 9);//set the software serial to use port 8 and 9 

void setup() {
  Serial.begin(9600);//use the default serial baud rate
  //robot tank setup pins
  pinMode(RIGHT_BACKWARD,OUTPUT);
  pinMode(RIGHT_FORWARD,OUTPUT);
  pinMode(LEFT_BACKWARD,OUTPUT);
  pinMode(LEFT_FORWARD,OUTPUT);
  Stop(); //stop the motors of the tank
//robot arm initialise
  pwm.begin();
  pwm.setPWMFreq(60);  // This is the maximum PWM frequency
  delay(10);
  //bluetooth
  mySerial.begin(9600);
  delay(1000);
  ///-----
  //initialise the position of the robot arm motors
  Wrist_Default_Position();
  Elbow_Default_Position();
  Shoulder_Default_Position();
  Waist_Default_Position();
  Hip_Default_Position();
  OpenCraw();
}
///////////////////
//Robot tank functions
void Stop()
{//stop all motors
  digitalWrite(RIGHT_BACKWARD,LOW);
  digitalWrite(RIGHT_FORWARD,LOW);
  digitalWrite(LEFT_FORWARD,LOW);
  digitalWrite(LEFT_BACKWARD,LOW);
}
void Move_Backward()
{//drive backward
  digitalWrite(LEFT_BACKWARD,HIGH);//move the left motor backward
  digitalWrite(RIGHT_BACKWARD,HIGH);//move the right motor backward
  delay(1000);
  Stop();
}
void Move_Forward()
{//drive forward
  digitalWrite(RIGHT_FORWARD,HIGH);//move the left motor forward
  digitalWrite(LEFT_FORWARD,HIGH);//move the right motor forward
  delay(1000);
  Stop();
}
///////////////////////////
//Robot ARM functions
void OpenCraw()
{
  Serial.println("open craw");
  pwm.setPWM(CRAW_SERVO, 0, SERVOMIN);//0 degree
  delay(1000);
}
void CloseCraw()
{
  Serial.println("close craw");
  pwm.setPWM(CRAW_SERVO, 0, SERVOMAX);
  delay(1000);
}

void Wrist_Default_Position()
{
  Serial.println("Wrist default position");
  pwm.setPWM(WRIST_SERVO, 0, SERVOMIN);
  delay(1000);
}
void Wrist_Rotate(int angle)
{//rotate the wrirst with angle - max 180
  if (angle > 180) angle=180;
  Serial.println("Wrist Rotate");
  uint16_t diff=SERVOMAX-SERVOMIN;
  double ratio=angle/180.0*diff;
  uint16_t pulselength=(uint16_t) ratio+SERVOMIN;
  pwm.setPWM(WRIST_SERVO, 0, pulselength);
  delay(1000);
}
void Elbow_Default_Position()
{
  Serial.println("Elbow default position");
  pwm.setPWM(ELBOW_SERVO, 0, SERVOMAX);
  delay(1000);
}
void Elbow_Rotate(int angle)
{//rotate the elbow with angle - max 180
 if (angle > 180) angle=180;
 angle=180-angle;
  Serial.println("Elbow Rotate");
  uint16_t diff=SERVOMAX-SERVOMIN;
  double ratio=angle/180.0*diff;
  uint16_t pulselength=(uint16_t) ratio+SERVOMIN;
  pwm.setPWM(ELBOW_SERVO, 0, pulselength);
  delay(1000);
}
void Shoulder_Default_Position()
{
  Serial.println("Shoulder default position");
  pwm.setPWM(SHOULDER_SERVO, 0, SERVOMAX);
  delay(1000);
}
void Shoulder_Rotate(int angle)
{//rotate the shoulder with angle - max 180
 if (angle > 180) angle=180;
 angle=180-angle;
  Serial.println("Shoulder Rotate");
  uint16_t diff=SERVOMAX-SERVOMIN;
  double ratio=angle/180.0*diff;
  uint16_t pulselength=(uint16_t) ratio+SERVOMIN;
  pwm.setPWM(SHOULDER_SERVO, 0, pulselength);
  delay(1000);
}
void Waist_Default_Position()
{
  Serial.println("Waist default position");
  //pwm.setPWM(WAIST_SERVO, 0, SERVOMAX);
  //pwm.setPWM(WAIST_SERVO, 0, 400);
  pwm.setPWM(WAIST_SERVO, 0, 580);
  delay(1000);
}

void Waist_Rotate(int angle)
{//rotate teh waist with angle - limit it to 130
 if (angle > 130) angle=130;
 angle=180-angle;
  Serial.println("Waist Rotate");
  uint16_t diff=SERVOMAX-SERVOMIN;
  double ratio=angle/180.0*diff;
  uint16_t pulselength=(uint16_t) ratio+SERVOMIN;
  pwm.setPWM(WAIST_SERVO, 0, pulselength);
  delay(1000);
}
void Hip_Default_Position()
{
  Serial.println("Hip default position");
  pwm.setPWM(HIP_SERVO,0,SERVOMIN);
}

void Hip_Rotate(int angle)
{//Rotate the hip with angle - max 180
  if (angle > 180) angle=180;
  Serial.println("Hip Rotate");
  uint16_t diff=SERVOMAX-SERVOMIN;
  double ratio=angle/180.0*diff;
  uint16_t pulselength=(uint16_t) ratio+SERVOMIN;
  pwm.setPWM(HIP_SERVO, 0, pulselength);
  delay(1000);
}

void loop() {
  val=0;
  if (mySerial.available())
  {//check if command is received from Bluetooth
    val = (char)mySerial.read();
  }
  switch (val)
  {
    case 'p':OpenCraw();break;
    case 'P':CloseCraw();break; 
    case 'w':Move_Forward();break;
    case 'x':Move_Backward();break;
    case 'o':Wrist_Rotate(0);break;
    case 'O':Wrist_Rotate(90);break;
    case 'i':Elbow_Default_Position();break;
    case 'I':Elbow_Rotate(180);break;
    case 'u':Shoulder_Default_Position();break;
    case 'U':Shoulder_Rotate(90);break;
    case 'y':Waist_Default_Position();break;
    case 'Y':Waist_Rotate(90);break;
    case 't':Hip_Default_Position();break;
    case 'T':Hip_Rotate(90);break;
  }
}
