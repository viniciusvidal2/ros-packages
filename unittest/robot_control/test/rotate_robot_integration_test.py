#! /usr/bin/env python

from robot_control.rotate_robot import RobotControl
from robot_control.srv import RotateRobot, RotateRobotRequest
from std_srvs.srv import Empty, EmptyRequest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
import rospy
import rosunit
import unittest
import rostest
import time
PKG = 'robot_control'
NAME = 'rotate_robot_integration_test'

class TestRobotControl(unittest.TestCase):

    def setUp(self):

        rospy.init_node('test_node')
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.init_orientation = Quaternion()
        self.current_orientation = Quaternion()
        self.init_yaw = 0
        self.final_yaw = 0
        self.service_call_reset()
        self.get_init_position()
        self.service_call()

    def get_init_position(self):

        rospy.wait_for_message("/odom", Odometry, timeout=10)
        self.init_orientation = self.current_orientation
        self.init_yaw = self.quaternion_to_euler(self.init_orientation)

    def odom_callback(self, msg):

        self.current_orientation = msg.pose.pose.orientation

    def quaternion_to_euler(self, msg):

        orientation_list = [msg.x, msg.y, msg.z, msg.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw

    def service_call(self):

        rospy.wait_for_service('rotate_robot')
        s = rospy.ServiceProxy('rotate_robot', RotateRobot)
        tests = [(60, 90, 'y')]

        for x, y, z in tests:
            print("Requesting %s+%s+%s" % (x, y, z))
            resp = s.call(RotateRobotRequest(x, y, z))


    def service_call_reset(self):

        rospy.wait_for_service('/gazebo/reset_world')
        s = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        resp = s.call(EmptyRequest())

    def test_correct_rotation(self):
        
        self.final_yaw = self.quaternion_to_euler(self.current_orientation)
        yaw_diff = self.init_yaw - self.final_yaw
        self.assertTrue((1.3 <= yaw_diff <= 2.1), "Integration error. Rotation was not between the expected values.")


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestRobotControl)