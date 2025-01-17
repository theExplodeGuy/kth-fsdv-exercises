#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64, Float32

def callback(data):
    rospy.loginfo("I heard " + str(data.data))
    pub = rospy.Publisher('/kthfsdv/result', Float32, queue_size=10)
    rospy.init_node('nodeB', anonymous=False)
    # rate = rospy.Rate(20)
    msg = data.data / 0.15
    pub.publish(msg)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('nodeB', anonymous=False)

    rospy.Subscriber("stavrou", Int64, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
