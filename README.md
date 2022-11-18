# ROS

Run in different terminals:
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0 (or whatever port)

cd exo_ws
source ./devel/setup.bash

rosrun servo_control publisher.py

To send angle run :
rostopic pub servo std_msgs/UInt16 1 (1 for 180 0 for 0)

