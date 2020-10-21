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
from skimage import color
import imutils
from pyefd import elliptic_fourier_descriptors
import matplotlib.pyplot as plt
from EFD_Calculator import EFD_Calculator
from std_msgs.msg import Float32MultiArray as FloatArray
from geometry_msgs.msg import Pose
import scipy as sp
from scipy import signal

        
class image_processor:

    def __init__(self):
        rospy.init_node('image_processor', anonymous=True)
        self.status = False
        self.EFD_publisher = rospy.Publisher('EFD_constants', FloatArray, queue_size= 1)
        self.Pose_publisher = rospy.Publisher('Frame_Pose', Pose, queue_size=1)
        self.EFD_msg = FloatArray()

        self.EFD_Calculator = EFD_Calculator(5)
        self.bridge = CvBridge()
        self.minBlobSize = rospy.get_param("min blob size")
        self.maxAutofillSize = rospy.get_param("max autofill size")

        rospy.Subscriber('acquired_image', Image, self.ProcessImage)

        # create thread
        self.feed_thread = Thread(target=self.FCD, args=())
        self.feed_thread.daemon = True
        self.feed_thread.start()
        rospy.loginfo("Thread created")
        # FPS setup
        self.FPS = 1 / (rospy.get_param('Desired_FPS'))
        self.FPS_MS = int(self.FPS * 1000)
        rospy.loginfo("FPS constants set")

        self.waitkeyval = 20

        #if not rospy.get_param('use Gazebo cam'):
        #    self.use_camera = rospy.get_param('use camera')
        #    if self.use_camera:
        #        self.waitkeyval = 20
        #    else:
        #        self.waitkeyval = 1


        while not rospy.is_shutdown():
            try:
                self.display_frame()
            except AttributeError:
                rospy.loginfo("Error, may be temporary")
                pass



    def FCD(self):
        pose = self.GetPose()
        if self.status:
            gray_img = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            self.bin_img = self.filter_image(gray_img)
            self.update_max_cnt()
            FCD_Coeffs = self.EFD_Calculator.calc_coeffs(self.max_contour)
            self.calc_cMax(FCD_Coeffs)
            print(FCD_Coeffs)
            self.EFD_msg.data = self.format_EFD_message_data(FCD_Coeffs)
            self.Pose_publisher.publish(pose)
            self.EFD_publisher.publish(self.EFD_msg)
            rospy.loginfo("New frame published")
            #self.cont_frame = self.bin_img
            time.sleep(self.FPS)

    def calc_cMax(self, coeffs):
        N = np.stack((coeffs[2], coeffs[3]))
        C = np.linalg.norm(N, axis=0)
        C_max = sp.signal.argrelextrema(C, np.greater)
        print("******")
        print("******")
        print("C max vals: ")
        print(C_max)

    def format_EFD_message_data(self, coeffs):
        return coeffs

    def update_max_cnt(self):
        # Get max contour
        contours = cv2.findContours(self.bin_img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        closed_contours = []
        for i in contours:
            if cv2.contourArea(i) > cv2.arcLength(i, True):
                closed_contours.append(i)
        self.max_contour = max(closed_contours, key=cv2.contourArea)
        out = np.zeros(self.bin_img.shape, np.uint8)
        cv2.drawContours(out, [self.max_contour], -1, 255, cv2.FILLED)
        bin_mask = cv2.bitwise_and(self.bin_img, out)

        masked_img = cv2.bitwise_and(self.frame, self.frame, mask = bin_mask)


        # calc center of contour
        M = cv2.moments(self.max_contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # draw contour and center
        cv2.drawContours(masked_img, [self.max_contour], -1, (0, 255, 0), 2)
        cv2.circle(masked_img, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(masked_img, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        self.cont_frame = masked_img


    def convert_to_cv2(self, data):
        rospy.loginfo("image received")
        self.frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')


    def ProcessImage(self, data):
        self.convert_to_cv2(data)
        self.status = True
        self.FCD()

    def GetPose(self):
        return Pose()

    def filter_image(self,img):
        bin_img = color.rgb2gray(img)
        #bin_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Adaptive threshold -> binary image
        gray_bi = cv2.medianBlur(bin_img, 5)
        #gray_bi = cv2.GaussianBlur(img, (5,5), 0)
        #gray_bi = cv2.threshold(gray_bi, 60, 255, cv2.THRESH_BINARY)[1]
        gray_bi = cv2.adaptiveThreshold(gray_bi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 25, 3)
        gray_bi_array = np.array(gray_bi)

        gray_size_filtered = gray_bi_array == 0

        #filter noise, fill gaps, and close contours
        gray_size_filtered = morphology.remove_small_objects(gray_size_filtered, min_size=self.minBlobSize)
        gray_size_filtered = morphology.remove_small_holes(gray_size_filtered, self.maxAutofillSize)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9))
        gray_size_filtered = gray_size_filtered.astype(np.uint8)  # convert to an unsigned byte
        gray_size_filtered *= 255

        gray_size_filtered = cv2.morphologyEx(gray_size_filtered, cv2.MORPH_CLOSE, kernel)
        # blob fill in
        bi_filled = gray_size_filtered.copy()
        h, w = gray_size_filtered.shape[:2]
        mask = np.zeros((h + 2, w + 2), np.uint8)
        cv2.floodFill(bi_filled, mask, (0, 0), 255)
        bi_filled_inv = cv2.bitwise_not(bi_filled)
        final = gray_size_filtered | bi_filled_inv
        return final

    def display_frame(self):
        """
        updates the view windows so the user can observe
        """
        try:
            cv2.imshow('frame',self.cont_frame)
            cv2.waitKey(self.waitkeyval)
        except:
            rospy.loginfo("No frame")



if __name__ == '__main__':
    imageProcessor = image_processor()
    rospy.spin()
    

