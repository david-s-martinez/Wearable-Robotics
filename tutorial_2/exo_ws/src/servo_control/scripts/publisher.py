#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, UInt16

def talker():
    pub = rospy.Publisher('servo', UInt16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        data = int(input())
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass