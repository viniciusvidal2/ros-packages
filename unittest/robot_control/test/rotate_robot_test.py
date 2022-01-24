#! /usr/bin/env python

from robot_control.rotate_robot import RobotControl
import rosunit
import unittest
import sys
PKG = 'robot_control'
NAME = 'rotate_robot_test'


class TestRobotControl(unittest.TestCase):

    def setUp(self):
        self.rc = RobotControl()

    # only functions with 'test_'-prefix will be run!
    def test_deg_rad_conversion(self):

        speed, angle = self.rc.convert_degree_to_rad(60, 90)
        self.assertEquals(angle, 1.57, "1.57!=1.57")
        self.rc.shutdownhook()


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestRobotControl)