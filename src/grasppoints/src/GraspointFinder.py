#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool as BoolMsg
import time
from std_msgs.msg import Float32MultiArray as FloatArray

class GraspointFinder:

    def __init__(self):
        rospy.init_node('GraspointMachine', anonymous=True)
        self.previousCurvature = 0
        self.previousPose = Pose()
        self.currentCurvature = 0
        self.currentPose = Pose()
        self.OptimumCurvature = 0
        self.OptimumPose = Pose()
        self.OOV = False
        self.distance = 80
        self.threshold = 10
        rospy.Subscriber('Curvature_Sums', float, self.update_curvatures)
        rospy.Subscriber('Frame_Pose', Pose, self.update_pose)
        rospy.Subscriber('OOV', BoolMsg, self.update_OOV)
        #rospy.Subscriber('Distance', ???, self.update_distance)


        #assumption made: object starts completely in view of camera


        self.optimize_direction("Right")
        self.move_to(self.OptimumPose)
        self.optimize_direction("Left")
        self.move_to(self.OptimumPose)
        self.optimize_direction("Up")
        self.move_to(self.OptimumPose)
        self.optimize_direction("Down")
        self.move_to(self.OptimumPose)

        #move forward
        while self.distance > self.threshold:
            time.sleep(10)
        #send empty trajectory
        #grab object


    def update_distance(self,data):
        self.distance = data.distance

    def update_pose(self,data):
        self.currentPose = data.Pose

    def update_OOV(self, Bool):
        self.OOV = Bool.data
        print("OUT OF BOUNDS UPDATED TO " + str(self.OOV))

    def move_to(self, pose):
        pass

    def optimize_direction(self, direction):
        if direction == "Right":
            #move right
            pass
        elif direction == "Left":
            # move left
            pass
        elif direction == "Up":
            # move up
            pass
        elif direction == "Down":
            # move down
            pass
        while self.currentCurvature >= self.previousCurvature & (not self.OOV):
            self.OptimumCurvature = self.currentCurvature
            self.OptimumPose = self.currentPose
            time.sleep(10)
        #send empty trajectory here to stop arm




    def update_curvatures(self, OptimizationMetric):
        self.previousCurvature = self.currentCurvature.copy()
        self.previousPose = self.currentPose
        self.currentCurvature = OptimizationMetric
