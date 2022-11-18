
#include <ros.h>

#include <ESP32Servo.h>
#include <std_msgs/Float64.h>

#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
 
#include <std_msgs/UInt16.h>
#include <string.h>
ros::NodeHandle nh;
Servo myservo;

#define SERVO_PIN 12

double posServo1 = 2100; //0
double posServo2 = 1160; //90

void servo_set( const std_msgs::Float32& cmd_msg){
  float degs = cmd_msg.data -30.0;
  float rads = degs * (3.1416/180.0);
  double angle = mapf(rads, 0.0,1.570796, posServo1, posServo2);
  myservo.writeMicroseconds(angle);
  
}
//
double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ros::Subscriber<std_msgs::Float32> sub("servo", servo_set);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  pinMode(SERVO_PIN, OUTPUT);

  myservo.attach(SERVO_PIN); // PWM pin
  delay(1000);
  
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
