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
from std_msgs.msg import Bool as BooleanMsg
from grasppoints.srv import CalcGrasppoints, CalcGrasppointsResponse


class GraspPointFilter:

    def __init__(self):
        print("Initializing Graspoint service...")
        rospy.init_node('GraspPointFilter', anonymous=True)
        grasppoint_service = rospy.Service('calc_grasppoints', CalcGrasppoints, self.calculate_grasppoints)

    def calculate_grasppoints(self, EFD_req):
        Nx = EFD_req.Nx
        Ny = EFD_req.Ny
        Tx = EFD_req.Tx
        Ty = EFD_req.Ty
        N = np.stack((Nx, Ny))
        C = np.linalg.norm(N, axis=0)
        C_max = sp.signal.argrelextrema(C, np.greater)
        Sum, Grasp = self.optimize(C, C_max, Tx, Ty)
        C_max = C_max[0]
        C = C.tolist()
        return CalcGrasppointsResponse(C, C_max, Sum, Grasp)

    def optimize(self, C, C_max, Tx, Ty):
        #T here is Tx and Ty stacked on each other
        T = np.stack((Tx, Ty))
        Concave = []
        for i in C_max[0]:
            cross = np.cross(T[:, i - 1], T[:, i])
            if cross > 0:
                Concave.append(i)
        Sum = 0
        if np.size(Concave) > 1:
            for i1 in Concave:
                for i2 in Concave:
                    if C[i1] + C[i2] > Sum and C[i1] != C[i2]:
                        Sum = C[i1] + C[i2]
                        Grasp = [i1, i2]
        else:
            for i1 in C_max[0]:
                for i2 in C_max[0]:
                    if C[i1] + C[i2] > Sum and C[i1] != C[i2]:
                        Sum = C[i1] + C[i2]
                        Grasp = [i1, i2]
        return Sum, Grasp



if __name__ == '__main__':
    Grasppointfilter = GraspPointFilter()
    rospy.spin()


