#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int64

def talker():
    pub = rospy.Publisher('stavrou', Int64, queue_size=10)
    rospy.init_node('nodeA', anonymous=False)
    rate = rospy.Rate(5) # 10hz
    msg = 1
    while not rospy.is_shutdown():
        msg += 1
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
