#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class image1_43_converter:

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

        # initialize a publisher to send joints' estimated position to topics
        # this camera views the robot on the y and z axes
        self.joint1_est_pub = rospy.Publisher(
            "/estimates/yz/joint1_4.3", Float64MultiArray, queue_size=10)
        self.joint2_est_pub = rospy.Publisher(
            "/estimates/yz/joint2_4.3", Float64MultiArray, queue_size=10)
        self.joint3_est_pub = rospy.Publisher(
            "/estimates/yz/joint3_4.3", Float64MultiArray, queue_size=10)
        self.joint4_est_pub = rospy.Publisher(
            "/estimates/yz/joint4_4.3", Float64MultiArray, queue_size=10)

        # initialize arrays to store joint positions
        self.joint1_pos = np.array([])
        self.joint2_pos = np.array([])
        self.joint3_pos = np.array([])
        self.joint4_pos = np.array([])

        # record start time
        self.time_trajectory = rospy.get_time()

  # receive data from camera 1, process it, and publish
    def callback1(self,data):
        # receive the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', self.cv_image1)

        self.find_joints(self.cv_image1)
        self.move_joints()

        im1=cv2.imshow('window1', self.cv_image1)
        cv2.waitKey(1)

        # publish the results
        try: 
            self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
        except CvBridgeError as e:
            print(e)

    # find the centres of the joints and publishes these to topics
    def find_joints(self, image):
        im=image
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 15, 255, cv2.THRESH_BINARY_INV)
        # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # im = cv2.drawContours(thresh, contours, -1,(0,0,255),1)
        # im_edge = cv2.Canny(thresh,100,200,apertureSize=5,L2gradient=True)
        # cv2.imshow('contours',im_edge)
        # cv2.waitKey(1)

        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, im.shape[0]/64, param1=200, param2=10, minRadius=5, maxRadius=30)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # Draw outer circle
                cv2.circle(im, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # Draw inner circle
                cv2.circle(im, (i[0], i[1]), 2, (0, 0, 255), 3)
            cv2.imshow('circles',circles)
            cv2.waitKey(1)
        # im_edge_all = cv2.Canny(gray,100,200,apertureSize=5,L2gradient=True)
        # im_and = cv2.bitwise_not(im_edge_gray)
        cv2.imshow('and',thresh)
        cv2.waitKey(1)

        # set up a blob detector to detect circles
        # p = cv2.SimpleBlobDetector_Params()
        # p.filterByCircularity = True
        # p.minCircularity = 0.2
        # p.filterByConvexity = False
        # p.filterByArea = False
        # p.filterByInertia = False
        # blob_det = cv2.SimpleBlobDetector_create(p)
        # keypoints = blob_det.detect(thresh)

        # # there should be one circle so one keypoint only.
        # if len(keypoints) > 0:
        #     h, w = 50, 50
        #     cx,cy = keypoints[0].pt
        #     sub_img = image[int(cy-h/2):int(cy+h/2), int(cx-w/2):int(cx+w/2)]
        #     M = cv2.moments(sub_img)
        #     cx = int(cx - w/2 + M["m10"] / M["m00"])
        #     cy = int(cy - h/2 + M["m01"] / M["m00"])
        #     blobs = cv2.drawMarker(image, (int(cx),int(cy)), (0,0,255))
        #     number_of_blobs = len(keypoints) 
        #     cv2.imshow("Filtering Circular Blobs Only", blobs) 


    # detecting the centre of the {colour} circle
    def detect_colour(self, joint, image, bgr_low, bgr_up):
        mask = cv2.inRange(image, bgr_low, bgr_up)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if M["m00"] != 0:   # this prevents division by zero when the colour is not visible.
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx,cy = joint   # return previous known position; cannot see joint
        return np.array([cx, cy])

    # moves joints using sin function
    def move_joints(self):
        t = rospy.get_time() - self.time_trajectory
        # ~~comment out the following 3 lines for the robot to run indefinitely~~
        if (t >= 5):    # reset time to 0 after 5 seconds
            t = 0
            self.time_trajectory = rospy.get_time()

        # calculate sinusoidal signals
        j2 = (np.pi / 2) * np.sin((np.pi/15) * t)
        j3 = (np.pi / 2) * np.sin((np.pi/18) * t)
        j4 = (np.pi / 2) * np.sin((np.pi/20) * t)
        # ~~change j4 pi/2 to pi/3 to prevent EE hitting ground if running longer than 5 seconds~~

        # publish the results
        self.robot_joint2_pub.publish(j2)
        self.robot_joint3_pub.publish(j3)
        self.robot_joint4_pub.publish(j4)

# call the class
def main(args):
    ic = image1_43_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
