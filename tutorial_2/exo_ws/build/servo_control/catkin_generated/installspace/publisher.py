#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, UInt16, Float32

def talker():
    pub = rospy.Publisher('servo', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        try:
            data = float(input())
            if 120.0 >= data>= 30.0:
            
            # if data:    
                rospy.loginfo(data)
                pub.publish(data)
                rate.sleep()
        except:
            print('Data not float')
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass