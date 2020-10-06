#!/usr/bin/env python

import rospy
from PIL import Image


class ImageAcquirer:

    def __init__(self):
        """
        Pulls image from camera
        Publishes image
        """

        # ROS initializations
        rospy.init_node('ImageAcquirer', anonymous = True)
        self.img_pub = rospy.Publisher('image')
        self.rate = rospy.Rate(50) #50 hz
        while not rospy.is_shutdown():
            self.pull_static_image()
            self.rate.sleep()

    def pull_static_image(self):
        """
        reads image from file and publishes on image topic
        """
        try:
            img = Image.open("calli.png")
            self.img_pub.publish(img)
        except IOError:
            print('Error reading file')

    def pull_camera_image(self):
        """
        pulls image from webcam and publishes on image topic
        """

if __name__ == '__main__':
    node = ImageAcquirer()
    rospy.spin()