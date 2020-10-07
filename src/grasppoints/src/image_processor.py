#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge 
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String



        
class image_processor:

    def __init__(self):
        rospy.init_node('image_processor', anonymous=True)

        rospy.Subscriber('acquired_image', Image, self.callback)

        self.bridge = CvBridge()


    def callback(self, data):
        rospy.loginfo("image received")
        #cv_image = self.bridge.imgmsg_to_

if __name__ == '__main__':
    imageProcessor = image_processor()
    rospy.spin()
    

