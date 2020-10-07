#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image
import cv2
import rospkg


class ImageAcquirer:

    def __init__(self):
        """
        Pulls image from camera
        Publishes image
        """

        # ROS initializations
        rospy.init_node('ImageAcquirer', anonymous = True)
        self.use_camera = rospy.get_param('use camera')
        self.img_pub = rospy.Publisher('acquired_image', Image, queue_size =10)
        self.bridge = CvBridge()


        if self.use_camera:
            self.vid = cv2.VideoCapture(0)
            while not rospy.is_shutdown():
                self.pull_camera_image()
                rospy.sleep(1.)
            self.vid.release
        else:
            while not rospy.is_shutdown():
                self.pull_static_image()
                rospy.sleep(1.)

    def pull_static_image(self):
        """
        reads image from file and publishes on image topic
        """
        rospack = rospkg.RosPack()
        image_path = rospack.get_path('grasppoints') + '/src/'
        try:
            cv_image = cv2.imread(image_path + 'calli.png', 0)
            image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
            self.img_pub.publish(image_message)
            #rospy.loginfo("Image published")
        except IOError:
            rospy.loginfo("Error reading file")

    def pull_camera_image(self):
        """
        pulls image from webcam and publishes on image topic
        """
        ret, frame = self.vid.read()
        cv2.imshow('frame',frame)
        key = cv2.waitKey(1)
        rospy.loginfo("Image displayed")


if __name__ == '__main__':
    node = ImageAcquirer()
    rospy.spin()