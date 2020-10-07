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


        if self.use_camera:
            #Setup camera
            self.vid = cv2.VideoCapture(0)
            self.vid.set(cv2.CAP_PROP_BUFFERSIZE,1)
            rospy.loginfo("Camera feed instantiated")

            #FPS setup
            self.FPS = 1/60
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
            self.pull_static_image()
            rospy.sleep(1.)
            cv2.imshow('Static image filtered', self.frame)
            cv2.waitKey(1)
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
            self.frame = cv2.imread(image_path + 'calli.jpg', 0)
            self.frame = self.filter_image(self.frame)
            image_message = self.bridge.cv2_to_imgmsg(self.frame, encoding="passthrough")
            self.img_pub.publish(image_message)
            rospy.loginfo("Image published")
            cv2.imshow('Static image filtered', self.frame)
            cv2.waitKey(1)
        except IOError:
            rospy.loginfo("Error reading file")

    def update_camera_image(self):
        """
        pulls image from webcam and publishes on image topic
        """
        while not rospy.is_shutdown():
            if self.vid.isOpened():
                (self.status, self.frame) = self.vid.read()
                img = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
                self.frame_filt = self.filter_image(img)
                self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.frame_filt, encoding="passthrough"))
                rospy.loginfo("New frame published")
            time.sleep(self.FPS)

    def display_frame(self):
        """
        updates the view windows so the user can observe
        """
        if self.status:
            cv2.imshow('frame',self.frame_filt)
            cv2.waitKey(20)
        else:
            rospy.loginfo("No frame")


    def filter_image(self,img):
        # Adaptive threshold -> binary image
        img = cv2.medianBlur(img, 5)
        gray_bi = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 25, 3)
        gray_bi_array = np.array(gray_bi)

        gray_size_filtered = gray_bi_array == 0

        gray_size_filtered = morphology.remove_small_objects(gray_size_filtered, min_size=50)
        gray_size_filtered = morphology.remove_small_holes(gray_size_filtered, 20)
        gray_size_filtered = gray_size_filtered.astype(np.uint8)  # convert to an unsigned byte
        gray_size_filtered *= 255

        # blob fill in
        bi_filled = gray_size_filtered.copy()
        h, w = gray_size_filtered.shape[:2]
        mask = np.zeros((h + 2, w + 2), np.uint8)
        cv2.floodFill(bi_filled, mask, (0, 0), 255)
        bi_filled_inv = cv2.bitwise_not(bi_filled)
        return gray_size_filtered | bi_filled_inv



if __name__ == '__main__':
    image_acquirer = ImageAcquirer()
    rospy.loginfo("Node created")
    rospy.spin()
