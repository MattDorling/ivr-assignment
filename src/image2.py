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
            "/estimates/xz/joint1", Float64MultiArray, queue_size=10)
        self.joint2_est_pub = rospy.Publisher(
            "/estimates/xz/joint2", Float64MultiArray, queue_size=10)
        self.joint3_est_pub = rospy.Publisher(
            "/estimates/xz/joint3", Float64MultiArray, queue_size=10)
        self.joint4_est_pub = rospy.Publisher(
            "/estimates/xz/joint4", Float64MultiArray, queue_size=10)

        # initialize a publisher to send target's estimated position to a topic
        self.target_est_pub = rospy.Publisher(
            "/estimates/xz/target", Float64MultiArray, queue_size=10)

        # initialize arrays to store joint positions
        self.joint1_pos = np.array([])
        self.joint2_pos = np.array([])
        self.joint3_pos = np.array([])
        self.joint4_pos = np.array([])
        self.target_pos = np.array([])

  # receive data from camera 2, process it, and publish
    def callback2(self,data):
        # receive the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)
        
        self.find_joints()
        self.detect_target()
        
        im2=cv2.imshow('window2', self.cv_image2)
        cv2.waitKey(1)
    
        # publish the results
        try: 
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
        except CvBridgeError as e:
            print(e)

    # detect the location of the target sphere
    def detect_target(self):
        # get orange colours only from image
        image = cv2.inRange(self.cv_image2,(5, 30, 20), (25, 255, 255) )

        # set up a blob detector to detect circles
        p = cv2.SimpleBlobDetector_Params()
        p.filterByCircularity = True
        p.minCircularity = 0.7
        p.filterByConvexity = False
        p.filterByArea = False
        p.filterByInertia = False
        blob_det = cv2.SimpleBlobDetector_create(p)
        keypoints = blob_det.detect(image)

        """
        the keypoints are not accurately in the centre of the circles, but are within the circles.
        to get the centre accurately, crop the image around the keypoint to isolate the target circle.
        then, use moments to get the centre of mass of the blob.
        """

        # there should be one circle so one keypoint only.
        if len(keypoints) == 1:
            h, w = 50, 50
            cx,cy = keypoints[0].pt
            sub_img = image[int(cy-h/2):int(cy+h/2), int(cx-w/2):int(cx+w/2)]
            M = cv2.moments(sub_img)
            cx = int(cx - w/2 + M["m10"] / M["m00"])
            cy = int(cy - h/2 + M["m01"] / M["m00"])

        # if there are 0 or more than 1 keypoint, the target position is unknown.
        else:
            # use previous detected position
            cx,cy = self.target_pos
        
        self.target_pos = np.array([cx, cy])
        centre = Float64MultiArray()
        centre.data = np.array([cx, cy])
        self.target_est_pub.publish(centre)

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

    # finds the centres of the joints and publishes these to topics
    def find_joints(self):
        a = Float64MultiArray()
        # find centre of joints by detecting colour:
        # yellow
        self.joint1_pos = self.detect_colour(self.joint1_pos, self.cv_image2,
                                            (0, 100, 100), (0, 255, 255))
        a.data = self.joint1_pos
        self.joint1_est_pub.publish(a)
        
        # blue
        self.joint2_pos = self.detect_colour(self.joint2_pos, self.cv_image2,
                                            (100, 0, 0), (255, 0, 0))
        a.data = self.joint2_pos
        self.joint2_est_pub.publish(a)
        
        # green
        self.joint3_pos = self.detect_colour(self.joint3_pos, self.cv_image2,
                                            (0, 100, 0), (0, 255, 0))
        a.data = self.joint3_pos
        self.joint3_est_pub.publish(a)
        
        # red
        self.joint4_pos = self.detect_colour(self.joint4_pos, self.cv_image2,
                                            (0, 0, 100), (0, 0, 255))
        a.data = self.joint4_pos
        self.joint4_est_pub.publish(a)

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
