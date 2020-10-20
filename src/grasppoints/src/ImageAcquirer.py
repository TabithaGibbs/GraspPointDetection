#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image
import cv2
import rospkg
from threading import Thread
import time
import numpy as np
from skimage import morphology


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
        self.minBlobSize = rospy.get_param("min blob size")
        self.maxAutofillSize = rospy.get_param("max autofill size")
        # FPS setup
        self.FPS = 1 / (rospy.get_param('Desired_FPS'))
        self.FPS_MS = int(self.FPS * 1000)
        rospy.loginfo("FPS constants set")


        if self.use_camera:
            #Setup camera
            self.vid = cv2.VideoCapture(0)
            self.vid.set(cv2.CAP_PROP_BUFFERSIZE,1)
            rospy.loginfo("Camera feed instantiated")



            #create thread
            #self.feed_thread = Thread(target=self.update_camera_image, args=())
            #self.feed_thread.daemon = True
            #self.feed_thread.start()
            #rospy.loginfo("Thread created")

            #No current frame
            self.status = False
            while not rospy.is_shutdown():
                try:
                    self.update_camera_image()
                except:
                    rospy.loginfo("Error")
                    pass
                time.sleep(self.FPS)
        else:
            #repeatedly grab static image and publish it
            filename = rospy.get_param('image file')
            rospack = rospkg.RosPack()
            self.image_path = rospack.get_path('grasppoints') + '/src/TestImages/' + filename
            while not rospy.is_shutdown():
                try:
                    self.pull_static_image()
                except:
                    rospy.loginfo("Error reading file")
                    pass
                time.sleep(self.FPS)

    def pull_static_image(self):
        """
        reads image from file and publishes on image topic
        """

        try:
            self.frame = cv2.imread(self.image_path, 1)
            image_message = self.bridge.cv2_to_imgmsg(self.frame, encoding="passthrough")
            self.img_pub.publish(image_message)
        except:
            rospy.loginfo("something is wrong")

    def update_camera_image(self):
        """
        pulls image from webcam and publishes on image topic
        """
        if self.vid.isOpened():
            (self.status, self.frame) = self.vid.read()
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, encoding="passthrough"))


if __name__ == '__main__':
    image_acquirer = ImageAcquirer()
    rospy.loginfo("Node created")
    rospy.spin()
