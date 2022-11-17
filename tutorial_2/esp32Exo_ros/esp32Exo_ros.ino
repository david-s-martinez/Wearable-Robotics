
#include <ros.h>

#include <ESP32Servo.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
 
#include <std_msgs/UInt16.h>
#include <string.h>
ros::NodeHandle nh;
Servo myservo;

#define SERVO_PIN 12

double posServo1 = 2500; //0
double posServo2 = -2500; //90

//double posServo2 = 1160; //90

 
void servo_set( const std_msgs::UInt16& cmd_msg){
  double toggle = 2;
  toggle = cmd_msg.data;
  char arr[sizeof(toggle)];
  memcpy(arr,&toggle,sizeof(toggle)); 
  printf(arr);
  if(toggle==1){
    double angle = mapf(0.0, 0.0,1.570796, posServo1, posServo2);
    myservo.writeMicroseconds(angle);
    toggle = 2;
  }
  if(toggle==0){
    double angle = mapf(0.785398, 0.0,1.570796, posServo1, posServo2); 
    myservo.writeMicroseconds(angle);
    toggle = 2;
  }
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_set);
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
