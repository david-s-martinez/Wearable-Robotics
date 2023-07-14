
#include <ros.h>

#include <Servo.h> 
#include <std_msgs/Float64.h>

#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32MultiArray.h>

 
#include <std_msgs/UInt16.h>
#include <string.h>
ros::NodeHandle nh;
Servo myservo, myservo1;

#define SERVO_PIN1 14
#define SERVO_PIN 12
float posServo1 = 2100; //0
float posServo2 = 1160; //90

float deg2rad(float degree){
    return (degree * 3.14159265359/180);

}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void servo_set( const std_msgs::Float32MultiArray& cmd_msg){
  float q1 = mapf(deg2rad(cmd_msg.data[0] -30.0), 0.0,1.570796, posServo1, posServo2);
  float q2 = mapf(deg2rad(cmd_msg.data[1] -30.0), 0.0,1.570796, posServo1, posServo2);
  
  myservo.writeMicroseconds(q1);
  myservo1.writeMicroseconds(q2);
}
//

ros::Subscriber<std_msgs::Float32MultiArray> sub("servo", servo_set);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(SERVO_PIN1, OUTPUT);

  myservo.attach(SERVO_PIN); // PWM pin
  myservo1.attach(SERVO_PIN1);
  delay(1000);
  
}

void loop()
{
  nh.spinOnce();
  delay(10);
}
