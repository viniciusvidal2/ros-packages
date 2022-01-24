#! /usr/bin/env python

from robot_control.rotate_robot import RobotControl
import unittest


class CaseA(unittest.TestCase):

    def setUp(self):
        self.rc = RobotControl()

    def runTest(self):
        
        speed, angle = self.rc.convert_degree_to_rad(60, 90)
        self.assertEquals(angle, 1.57, "1.57!=1.57")
        self.rc.shutdownhook()


class CaseB(unittest.TestCase):

    def setUp(self):
        self.rc = RobotControl()

    def runTest(self):

        speed, angle = self.rc.convert_degree_to_rad(60, -90)
        self.assertEquals(angle, 1.57, "1.57!=1.57")
        self.rc.shutdownhook()


class MyTestSuite(unittest.TestSuite):

    def __init__(self):
        super(MyTestSuite, self).__init__()
        self.addTest(CaseA())
        self.addTest(CaseB())