#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import time


class GraspointFinder:

    def __init__(self):
        rospy.init_node('GraspointMachine', anonymous=True)
        self.previousCurvature = 0
        self.previousPose = Pose()
        self.currentCurvature = 0
        self.currentPose = Pose()
        self.OptimumCurvature = 0
        self.OptimumPose = Pose()
        rospy.Subscriber('Curvature_Sums', float, self.update_curvatures)
        rospy.Subscriber('Frame_Pose', Pose, self.update_pose)

        self.optimize_direction("Right")
        self.move_to(self.OptimumPose)
        self.optimize_direction("Left")
        self.move_to(self.OptimumPose)
        self.optimize_direction("Up")
        self.move_to(self.OptimumPose)
        self.optimize_direction("Down")
        self.move_to(self.OptimumPose)

    def update_pose(self,data):
        self.currentPose = data.Pose

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
        while self.currentCurvature >= self.previousCurvature:
            time.sleep(10)
        self.OptimumPose = self.previousPose




    def update_curvatures(self, OptimizationMetric):
        self.previousCurvature = self.currentCurvature.copy()
        self.previousPose = self.currentPose
        self.currentCurvature = OptimizationMetric
