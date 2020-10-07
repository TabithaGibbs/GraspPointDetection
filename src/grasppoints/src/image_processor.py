#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge 
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


def callback(data):
    rospy.loginfo("image received")
        
def image_processor():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('image_processor', anonymous=True)

    rospy.Subscriber('acquired_image', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    rospy.loginfo("running")

if __name__ == '__main__':
    image_processor()
    

