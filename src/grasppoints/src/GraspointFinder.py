#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose


class GraspointFinder:

    def __init__(self):
        rospy.init_node('GraspointMachine', anonymous=True)
        self.currentCurvature = 0
        self.currentPose = Pose()
        self.OptimumPose = Pose()
        rospy.Subscriber('OptimizationMetric', float, self.update_curvatures())


        self.optimize_direction("Right")




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



    def update_curvatures(self, OptimizationMetric):
        self.previousCurvature = self.currentCurvature.copy()
        self.previousPose = self.currentPose
        self.currentCurvature = OptimizationMetric
        self.currentPose
