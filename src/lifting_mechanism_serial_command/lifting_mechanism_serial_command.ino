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

   pot_read();
   state = Serial.read();
   switchstate = digitalRead(limit_switch);

   if (state == '1')
   {
    extend();
   }
   if (state == '2')
   {
    retract();
   }
   if (state =='3'){
    angle=23;
    Serial.print("Angle Value: ");
    Serial.println(angle);
    myServo.write(angle);
    delay(1000);
   }
   if(state=='4'){
     angle=100;
    Serial.print("Angle Value: ");
    Serial.println(angle);
    myServo.write(angle);
    delay(1000);
   }
   if (state == '5')
   {
    i=5;
   }
   if (state == '6')
   {
    i=6;
   }
   if (state == '7'){
    i=7;
   }
   
   else {
    analogWrite(PWM, (int)(0));
    myServo.write(0);
    angle=0;
   }

   if(i==5){
    drop();
   }
   if(i==6){
    lift();
   }
   
   if(pot_val<keep_lim and i==7){
    lift();
    i++;
   }
   if(pot_val>keep_lim and i==7){
    drop();
    i++;
   }
   if( pot_val == keep_lim and i==8){
    Serial.println("keeping position done!");
    Serial.println(pot_val);
    analogWrite(PWM_Driver, (int)(0));
    i=0;
   }
   
   if (pot_val < low_lim and i == 5)
   {
    Serial.println("Dropping done!");
    Serial.println(pot_val);
    analogWrite(PWM_Driver, (int)(0));
    i=0;
   }
   else if (pot_val > high_lim and i == 6)
   {
    Serial.println("Lifting done!");
    Serial.println(pot_val);
    analogWrite(PWM_Driver, (int)(0));
    i=0;
   }
   if(switchstate == HIGH){
  //Serial.print("safe");
  }
  else if(switchstate == LOW){
  //Serial.println("Not Safe");
  analogWrite(PWM_Driver, (int)(0));
  i=0;
  }

}



