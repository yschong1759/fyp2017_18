/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>

int pin_number = 8;

ros::NodeHandle  nh;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(pin_number, HIGH-digitalRead(pin_number));   // blink the led
  //Serial.println(toggle_msg.data);
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{ 
  pinMode(pin_number, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{ 
  
  nh.spinOnce();
  delay(1);
}

