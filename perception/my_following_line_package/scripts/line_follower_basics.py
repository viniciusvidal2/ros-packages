#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)

    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            # We get image dimensions and crop the parts of the image we dont need
            # Bear in mind that because its image matrix first value is start and second value is down limit.
            # Select the limits so that they get the line not too close, not too far, and the minimum portion possible
            # To make the process faster.
            height, width, channels = cv_image.shape
            descentre = 160
            rows_to_watch = 20
            crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
            # Convert from RGB to HSV
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

            # Define the Yellow Colour in HSV
            #RGB
            #[[[222,255,0]]]
            #BGR
            #[[[0,255,222]]]
            """
            To know which color to track in HSV, Put in BGR. Use ColorZilla to get the color registered by the camera
            >>> yellow = np.uint8([[[B,G,R ]]])
            >>> hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
            >>> print( hsv_yellow )
            [[[ 34 255 255]]
            """
            lower_yellow = np.array([20,100,100])
            upper_yellow = np.array([50,255,255])

            # Threshold the HSV image to get only yellow colors
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(crop_img,crop_img, mask= mask)

            # Calculate centroid of the blob of binary image using ImageMoments
            m = cv2.moments(mask, False)
            try:
                cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            except ZeroDivisionError:
                cy, cx = height/2, width/2
        
            # Draw the centroid in the resultut image
            # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
            cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)

            cv2.imshow("Original", cv_image)
            cv2.imshow("HSV", hsv)
            cv2.imshow("MASK", mask)
            cv2.imshow("RES", res)

            cv2.waitKey(1)
        
            cv2.circle(res,(centre_cicle_x, centre_cicle_y), LineWidth,(BGRColour of line),TypeOfLine)

            error_x = cx - width / 2;
            twist_object = Twist();
            twist_object.linear.x = 0.2;
            twist_object.angular.z = -error_x / 100;
            rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
            # Make it start turning
            self.movekobuki_object.move_robot(twist_object)
        
        
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)


def main():
    line_follower_object = LineFollower()
    rospy.init_node('line_following_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()