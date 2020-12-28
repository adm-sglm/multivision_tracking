#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, DeleteModel, SpawnModel


class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        # self.set_stage()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def set_stage(self):
        delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        print(delete_model_srv(model_name="mobile_base"))

    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", self.process_image(cv_image))
        cv2.waitKey(1)

    def process_image(self, cv_img):
        height, width, channels = cv_img.shape
        descentre = 160
        rows_to_watch = 20
        crop_img = cv_img[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        cx, cy = self.calculate_centroid(mask)        
        res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
        cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)        
        self.move_robot(cx, mask.shape[1])
        return res

    def calculate_centroid(self, mask):
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = mask.shape[0]/2, mask.shape[1]/2
        return cx, cy

    def move_robot(self, cx, width):
        # we calculate the difference of the middle point of the blob and the image
        difference_x = cx - width / 2
        print(difference_x)
        twist = Twist()
        # we put a bit of a linear speed so it approaches to target point
        twist.linear.x = 0.15
        twist.angular.z = -difference_x / 100
        print(twist)
        self.cmd_vel_pub.publish(twist)
        # twist.linear.


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