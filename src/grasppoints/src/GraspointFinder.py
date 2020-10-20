#!/usr/bin/env python

import rospy


class GraspointFinder:

    def __init__(self):
        rospy.init_node('GraspointMachine', anonymous=True)
        self.currentCurvature = 0
        rospy.Subscriber('OptimizationVal', float, self.update_curvatures())




    def optimize_direction(self, direction):
        self.optimize_direction()
        pass

    def update_curvatures(self, data):
        self.previousCurvature = self.currentCurvature
        self.currentCurvature = data