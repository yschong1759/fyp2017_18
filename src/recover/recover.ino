#include <Servo.h>

Servo myServo;
const int DIR = 8;//for linear actuator
const int Magnet_switch=12;
const int PWM = 11;// for linear actuator
const int DIR_Driver= 2;//for dc motor
const int PWM_Driver= 3;//for dc motor
const int SLP_Driver= 4;//for dc motor
const int limit_switch=7;
const int POT = A0;
const int high_lim = 670;//670
const int keep_lim = 420;//420
const int low_lim = 285;//285


int duty_cycle = 60; 
int state;// 1= extend link, 2=retract link , 3= extend plate, 4= retract plate, 5= dropping, 6=lifting, 7= keeping
int angle=0;
int duty_cycle_driver= 40; //percentage 40
int pot_val;
int i=0;
int switchstate=0;

//procedue ==> 1 -> 5 -> move under chair -> 4 -> 6/(7) -> move to location -> 5 -> 3 -> move out of chair -> 7 -> 2;

void setup() {
  pinMode(DIR, OUTPUT);
  pinMode(Magnet_switch,OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(DIR_Driver, OUTPUT);
  pinMode(PWM_Driver, OUTPUT);
  pinMode(SLP_Driver, OUTPUT);
  pinMode(POT, INPUT);
  pinMode(limit_switch,INPUT);
  myServo.attach(9);
  digitalWrite(Magnet_switch,LOW);
  digitalWrite(SLP_Driver, HIGH);
  Serial.begin(9600);
  Serial.println("Actuator initialized");
  Serial.println("Lock initialise");
  Serial.println("Servo initialise");
  Serial.println("Motor initialized");
}

int pot_read()
{
  pot_val = analogRead(POT);
  //if (Serial.available()>0){
  Serial.println(pot_val);
  
  //}
  
}

void retract()
{
  Serial.println("unLocking");
  digitalWrite(Magnet_switch,LOW);
  delay(1000);
  Serial.println("Retracting...");
  digitalWrite(DIR,LOW);
  analogWrite(PWM, (int)(duty_cycle*255/100));
  delay(5000);
  //state=0;
  Serial.println("Retract Finish");
}

void extend()
{

  Serial.println("Extend...");
  digitalWrite(DIR, HIGH);
  analogWrite(PWM, (int)(duty_cycle*255/100));
  delay(5000);
 // state=0;
  Serial.println("Extend Finish");
  Serial.println("Locking");
  digitalWrite(Magnet_switch,HIGH);
}

void drop()
{
  Serial.println("Dropping...");
  digitalWrite(DIR_Driver, LOW);
  analogWrite(PWM_Driver, (int)(duty_cycle_driver*255/100));
}

void lift()
{

  Serial.println("Lifting...");
  digitalWrite(DIR_Driver, HIGH);
  analogWrite(PWM_Driver, (int)(duty_cycle_driver*255/100));
}

void loop() {
  // put your main code here, to run repeatedly:
  pot_read();
  if(pot_val < 400) {
    Serial.println("Dropping");
    drop();
  } else {
    analogWrite(PWM_Driver, (int)(0));
  }

}
