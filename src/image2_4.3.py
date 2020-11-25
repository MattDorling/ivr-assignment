#!/usr/bin/env python3
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError

class image2_converter:

  # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send images from camera2 to a topic named image_topic2
        self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        # initialize a publisher to send joints' estimated position to topics
        # this camera views the robot on the x and z axes
        self.joint1_est_pub = rospy.Publisher(
            "/estimates/xz/joint1_43", Float64MultiArray, queue_size=10)
        self.joint2_est_pub = rospy.Publisher(
            "/estimates/xz/joint2_43", Float64MultiArray, queue_size=10)
        self.joint3_est_pub = rospy.Publisher(
            "/estimates/xz/joint3_43", Float64MultiArray, queue_size=10)
        self.joint4_est_pub = rospy.Publisher(
            "/estimates/xz/joint4_43", Float64MultiArray, queue_size=10)

        # initialize arrays to store joint positions
        self.joint1_pos = np.array([400,532])
        self.joint2_pos = np.array([400,477])
        self.joint3_pos = np.array([400,380])
        self.joint4_pos = np.array([400,300])

        # record start time
        self.time_trajectory = rospy.get_time()

  # receive data from camera 2, process it, and publish
    def callback2(self,data):
        # receive the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)
        
        # t = rospy.get_time() - self.time_trajectory
        # # ~~comment out the following 4 lines if the robot is running indefinitely~~
        # if (t >= 5):    # reset time to 0 after 5 seconds
        #     t = 0
        #     self.joint3_pos = np.array([400,380])
        #     self.joint4_pos = np.array([400,300])

        self.find_joints(self.cv_image2)
        
        im2=cv2.imshow('window2', self.cv_image2)
        cv2.waitKey(1)
    
        # publish the results
        try: 
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
        except CvBridgeError as e:
            print(e)

    # find the centres of the joints and publishes these to topics
    def find_joints(self, image):
        im=image.copy()

        # get binary image including only the black robot
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 15, 255, cv2.THRESH_BINARY_INV)

        # detect circles from binary image using Hough Transform
        circles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT, 1, im.shape[0]/64, param1=200, param2=5.5, minRadius=6, maxRadius=11)
        
        # get centres of circles
        centres = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for c in circles[0,:]:
                centres.append((c[0],c[1]))

        # remove circles from outside the black robot and circles that are too close to the static joints 1 & 2
        centres = self.remove_noise_circles(centres,thresh)

        # choose the nearest circle centre to the previous joint4 position
        joint4_candidate = self.find_nearest(centres,self.joint4_pos)
        # if there is a viable circle centre, select it
        if joint4_candidate is not None:
            self.joint4_pos = joint4_candidate
            # remove it from list so that it is not also selected for joint3
            centres.remove(joint4_candidate)

        # choose the nearest circle centre to the previous joint3 position
        joint3_candidate = self.find_nearest(centres,self.joint3_pos)
        # if there is a viable circle centre, select it
        if joint3_candidate is not None:
            self.joint3_pos = joint3_candidate

        # check if the detected joints have wrongly swapped positions, swap them back 
        if self.distance(self.joint2_pos, self.joint3_pos) > self.distance(self.joint2_pos,self.joint4_pos):
            temp = self.joint4_pos
            self.joint4_pos = self.joint3_pos
            self.joint3_pos = temp

        # drawing the detected positions - red marker is EE, green is joint 3 (corresponding to previous joint colours)
        cv2.drawMarker(image, (int(self.joint3_pos[0]),int(self.joint3_pos[1])),(0,255,0))
        cv2.drawMarker(image, (int(self.joint4_pos[0]),int(self.joint4_pos[1])),(0,0,255))

        # publishing the positions
        a = Float64MultiArray()
        a.data = self.joint3_pos
        self.joint3_est_pub.publish(a)
        a.data = self.joint4_pos
        self.joint4_est_pub.publish(a)

    # removing undesirable circle centres from list of centres
    def remove_noise_circles(self, centres, img):
        new_centres = []
        for c in centres:
            d1 = self.distance(c, self.joint1_pos)
            d2 = self.distance(c, self.joint2_pos)
            # if each centre is not too close to either of the static joints or outside of the black robot, include it.
            if d1 > 15 and d2 > 15 and img[c[1],c[0]] != 0:
                new_centres.append(c)
        return new_centres

    # find the nearest point to a given previous point
    def find_nearest(self, centres, prev):
        if len(centres) == 0:
            return None
        closest = centres[0]
        for c in centres:
            if self.distance(c,prev) < self.distance(closest,prev):
                closest = c
        # if self.distance(closest,prev) > 40: #TODO check this constant
        #     return None
        else:
            return closest

    # distance between two points
    def distance(self, pos_1, pos_2):
        pos_1 = np.array([pos_1[0],pos_1[1]])
        pos_2 = np.array([pos_2[0],pos_2[1]])
        return np.sqrt(np.sum((pos_1 - pos_2)**2))

# call the class
def main(args):
    ic = image2_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
