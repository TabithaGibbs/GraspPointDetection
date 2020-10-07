#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image
import cv2
import rospkg
from threading import Thread
import time


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
            #Setup camera
            self.vid = cv2.VideoCapture(0)
            self.vid.set(cv2.CAP_PROP_BUFFERSIZE,2)
            rospy.loginfo("Camera feed instantiated")

            #FPS setup
            self.FPS = 1/30
            self.FPS_MS = int(self.FPS * 1000)
            rospy.loginfo("FPS constants set")

            #create thread
            self.feed_thread = Thread(target=self.update_camera_image, args=())
            self.feed_thread.daemon = True
            self.feed_thread.start()
            rospy.loginfo("Thread created")

            #No current frame
            self.status = False

            while not rospy.is_shutdown():
                try:
                    self.display_frame()
                except AttributeError:
                    rospy.loginfo("Error")
                    pass
        else:
            #repeatedly grab static image and publish it
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
            rospy.loginfo("Image published")
        except IOError:
            rospy.loginfo("Error reading file")

    def update_camera_image(self):
        """
        pulls image from webcam and publishes on image topic
        """
        while not rospy.is_shutdown():
            if self.vid.isOpened():
                (self.status, self.frame) = self.vid.read()
                self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, encoding="passthrough"))
                rospy.loginfo("New frame published")
            time.sleep(self.FPS)

    def display_frame(self):
        """
        updates the view windows so the user can observe
        """
        if self.status:
            cv2.imshow('frame',self.frame)
            cv2.waitKey(1)
        else:
            rospy.loginfo("No frame")


if __name__ == '__main__':
    image_acquirer = ImageAcquirer()
    rospy.loginfo("Node created")
    rospy.spin()
