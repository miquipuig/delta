//per ara execució realitzant: 
//rostopic pub servo1 std_msgs/UInt16  <angle1> & rostopic pub servo2 std_msgs/UInt16  <angle2> & rostopic pub servo3 std_msgs/UInt16  <angle3>
//part del publisher ANGLE comentada
//rostopic pub servo std_msgs/UInt16MultiArray '{data: [<angle1>, <angle2>,<angle3>]}'

#if defined (ARDUINO) && (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/UInt16MultiArray.h"
//std_msgs::UInt16 angle;


ros::NodeHandle  nh;

Servo servo1;
Servo servo2;
Servo servo3;
std_msgs::Bool pushed_msg;

void servo_cb( const std_msgs::UInt16MultiArray&  cmd_msg){
  servo1.write(cmd_msg.data[0]); //set servo angle, should be from 0-180  
  servo2.write(cmd_msg.data[1]); 
  servo3.write(cmd_msg.data[2]);
}

const int button_pin = 7;
const int led_pin = 13;
bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;
//void updateAngle()
//{
//  angle.data = analogRead();
//}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo", servo_cb);

ros::Publisher pub_button("pushed", &pushed_msg);

void setup(){
 
  nh.initNode();

  nh.subscribe(sub);
  nh.advertise(pub_button);
  
  
  servo1.attach(9); //attach it to pin 9
  servo2.attach(10); //attach it to pin 10
  servo3.attach(11); //attach it to pin 11
  servo1.write(130);
  servo2.write(130);
  servo3.write(130);

  pinMode(led_pin, OUTPUT);
  pinMode(button_pin, INPUT);

  digitalWrite(button_pin, HIGH);
  last_reading = ! digitalRead(button_pin);

}

void loop(){
  nh.spinOnce();
 
  bool reading = ! digitalRead(button_pin);
    if (last_reading!= reading){
      last_debounce_time = millis();
      published = false;
  }
    if ( !published && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(led_pin, reading);
    pushed_msg.data = reading;
    pub_button.publish(&pushed_msg);
    published = true;
  }

  last_reading = reading;
   delay(20);
}
