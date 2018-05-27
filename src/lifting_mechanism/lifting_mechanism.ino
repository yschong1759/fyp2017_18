#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <Servo.h>

Servo myServo;
const int DIR = 8;    // for linear actuator
const int Magnet_switch = 12;
const int PWM = 11;  // for linear actuator
const int DIR_Driver = 2;    // for dc motor
const int PWM_Driver = 3;    // for dc motor
const int SLP_Driver = 4;    // for dc motor
const int limit_switch = 7;
const int POT = A0;
const int high_lim = 670;    // 670
const int keep_lim = 450;    // changed from 420
const int low_lim = 305;    // changed from 285


int duty_cycle = 60; 
int state = 7;    // 1=extend link, 2=retract link, 3=retract plate, 4=extend plate, 5=dropping, 6=lifting, 7=keeping
int angle = 0;
int duty_cycle_driver = 40; //percentage 40
int pot_val;
int i = 0;
int switchstate = 0;

/* For ROS */
ros::NodeHandle nh;
std_msgs::String message_string;
std_msgs::Int8 message_int;
ros::Publisher status_pub("lifting_mechanism_status", &message_string);
ros::Publisher output_pub("lifting_mechanism_message_output", &message_int);

void post_info(char text[]) {
  message_string.data = text;
  status_pub.publish(&message_string);
}

void post_status(int status) {
  message_int.data = status;
  output_pub.publish(&message_int);
}

void retrieve_msg(const std_msgs::Int8& latest_message) {
  state = latest_message.data;
}

ros::Subscriber<std_msgs::Int8> sub("lifting_mechanism_message_input", &retrieve_msg);

// procedure ==> 1 -> 5 -> move under chair -> 4 -> 6/(7) -> move to location -> 5 -> 3 -> move out of chair -> 7 -> 2;

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

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(status_pub);
  nh.advertise(output_pub);

}

int pot_read() {
  pot_val = analogRead(POT);
}

void retract() {
  post_info("unLocking");
  digitalWrite(Magnet_switch,LOW);
  delay(1000);
  post_info("Retracting...");
  digitalWrite(DIR,LOW);
  analogWrite(PWM, (int)(duty_cycle*255/100));
  delay(5000);
  //state=0;
  post_info("Retract Finish");
  post_status(1);
}

void extend() {
  post_info("Extend...");
  digitalWrite(DIR, HIGH);
  analogWrite(PWM, (int)(duty_cycle*255/100));
  delay(5000);
  // state=0;
  post_info("Extend Finish");
  post_info("Locking");
  digitalWrite(Magnet_switch,HIGH);
  post_status(2);
}

void drop() {
  post_info("Dropping...");
  digitalWrite(DIR_Driver, LOW);
  analogWrite(PWM_Driver, (int)(duty_cycle_driver*255/100));
}

void lift() {
  post_info("Lifting...");
  digitalWrite(DIR_Driver, HIGH);
  analogWrite(PWM_Driver, (int)(duty_cycle_driver*255/100));
}

void loop() {

  pot_read();
  //state = Serial.read();
  switchstate = digitalRead(limit_switch);

  if(switchstate == HIGH) {
    post_status(100);
  //Serial.print("safe");
  } else if(switchstate == LOW) {
    //Serial.println("Not Safe");
    analogWrite(PWM_Driver, (int)(0));
    state = 0;
    post_status(-1);
    post_info("Not safe!");
  }

  if (state == 0) {
    analogWrite(PWM_Driver, (int)(0));
  }

  if (state == 1) {    // extend link
    extend();
  }
  if (state == 2) {    // retract link
    retract();
  }
  if (state == 3) {   // extend plate
    angle = 23;
    //Serial.print("Angle Value: ");
    //Serial.println(angle);
    myServo.write(angle);
    delay(1000);
    post_status(3);
  }
  if(state == 4) {   // retract plate
    angle = 100;
    //Serial.print("Angle Value: ");
    //Serial.println(angle);
    myServo.write(angle);
    delay(1000);
    post_status(4);
  }
  if (state == 5) {    // dropping
    drop();
  }
  if (state == 6) {    // lifting
    lift();
  }
  if (state == 7) {    // keeping
    if(pot_val < keep_lim) {
      lift();
    }
    if(pot_val > keep_lim) {
      drop();
    }
  } else {
    analogWrite(PWM, 0);
    myServo.write(0);
    angle = 0;
  }

  if(abs(pot_val - keep_lim) < 5 and state == 7) {
    post_info("keeping position done!");
    //Serial.println(pot_val);
    analogWrite(PWM_Driver, (int)(0));
    state = 0;
    post_status(7);
  }

  if (pot_val < low_lim and state == 5) {
    post_info("Dropping done!");
    //Serial.println(pot_val);
    analogWrite(PWM_Driver, (int)(0));
    post_status(5);
    state = 0;
  } else if (pot_val > high_lim and state == 6) {
    post_info("Lifting done!");
    //Serial.println(pot_val);
    analogWrite(PWM_Driver, (int)(0));
    state = 0;
    post_status(6);
  }

  nh.spinOnce();
  delay(25);
}
