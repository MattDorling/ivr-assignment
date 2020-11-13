#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64, Time
from cv_bridge import CvBridge, CvBridgeError


class image1_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint1_pub = rospy.Publisher(
            "/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher(
            "/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher(
            "/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher(
            "/robot/joint4_position_controller/command", Float64, queue_size=10)

        # initialize a publisher to send joints' estimated position to a topic called joints_est_pos
        # this camera views the robot on the y and z axes
        self.joint1_est_pub = rospy.Publisher(
            "/estimates/yz/joint1", Float64MultiArray, queue_size=10)
        self.joint2_est_pub = rospy.Publisher(
            "/estimates/yz/joint2", Float64MultiArray, queue_size=10)
        self.joint3_est_pub = rospy.Publisher(
            "/estimates/yz/joint3", Float64MultiArray, queue_size=10)
        self.joint4_est_pub = rospy.Publisher(
            "/estimates/yz/joint4", Float64MultiArray, queue_size=10)

        self.joint1_pos = np.array([])
        self.joint2_pos = np.array([])
        self.joint3_pos = np.array([])
        self.joint4_pos = np.array([])

        self.time_trajectory = rospy.get_time()

        

  # Recieve data from camera 1, process it, and publish
    def callback1(self,data):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Uncomment if you want to save the image
        #cv2.imwrite('image_copy.png', cv_image)

        self.find_joints()
        self.move_joints()

        im1=cv2.imshow('window1', self.cv_image1)
        cv2.waitKey(1)
        
        # Publish the results
        try: 
            self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
        except CvBridgeError as e:
            print(e)


    # finds the centres of the joints and publishes these to topics
    def find_joints(self):
        a = Float64MultiArray()
        # find centre of joints by detecting colour:
        self.joint1_pos = self.detect_colour(self.joint1_pos, self.cv_image1,
                                            (0, 100, 100), (0, 255, 255))  # yellow
        a.data = self.joint1_pos
        self.joint1_est_pub.publish(a, )
        
        self.joint2_pos = self.detect_colour(self.joint2_pos, self.cv_image1,
                                            (100, 0, 0), (255, 0, 0))  # blue
        a.data = self.joint2_pos
        self.joint2_est_pub.publish(a)
        
        self.joint3_pos = self.detect_colour(self.joint3_pos, self.cv_image1,
                                            (0, 100, 0), (0, 255, 0))  # green
        a.data = self.joint3_pos
        self.joint3_est_pub.publish(a)
        
        self.joint4_pos = self.detect_colour(self.joint4_pos, self.cv_image1,
                                            (0, 0, 100), (0, 0, 255))  # red
        a.data = self.joint4_pos
        self.joint4_est_pub.publish(a)
    
    # Detecting the centre of the {colour} circle
    def detect_colour(self, joint, image, bgr_low, bgr_up):
        mask = cv2.inRange(image, bgr_low, bgr_up)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if M["m00"] != 0:   # this prevents division by zero when the colour is not visible.
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:   # TODO look again at this
            cx,cy = joint   # return previous known position; cannot see joint
        return np.array([cx, cy])

    # moves joints using sin function
    def move_joints(self):
        t = rospy.get_time() - self.time_trajectory
        j2 = (np.pi / 2) * np.sin((np.pi/15) * t)
        j3 = (np.pi / 2) * np.sin((np.pi/18) * t)
        j4 = (np.pi / 2) * np.sin((np.pi/20) * t)


        # Publish the results
        self.robot_joint2_pub.publish(j2)
        self.robot_joint3_pub.publish(j3)
        self.robot_joint4_pub.publish(j4)

# call the class
def main(args):
    ic = image1_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


