#!/usr/bin/env python3
import image1
import image2
import sys
import numpy as np
import rospy
import cv2
import message_filters
from std_msgs.msg import Float64MultiArray, Float64

class vision:
    def __init__(self):

        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        self.im1 = image1.image1_converter()
        self.im2 = image2.image2_converter()

        # image1 position topics
        self.yz_joint1_sub = rospy.Subscriber("/estimates/yz/joint1", Float64MultiArray, callback)
        self.yz_joint2_sub = rospy.Subscriber("/estimates/yz/joint2", Float64MultiArray, callback)
        self.yz_joint3_sub = rospy.Subscriber("/estimates/yz/joint3", Float64MultiArray, callback)
        self.yz_joint4_sub = rospy.Subscriber("/estimates/yz/joint4", Float64MultiArray, callback)

        # image2 position topics
        self.xz_joint1_sub = rospy.Subscriber("/estimates/xz/joint1", Float64MultiArray)
        self.xz_joint2_sub = rospy.Subscriber("/estimates/xz/joint2", Float64MultiArray)
        self.xz_joint3_sub = rospy.Subscriber("/estimates/xz/joint3", Float64MultiArray)
        self.xz_joint4_sub = rospy.Subscriber("/estimates/xz/joint4", Float64MultiArray)

        ts = message_filters.TimeSynchronizer([
            self.yz_joint1_sub,
            self.yz_joint2_sub,
            self.yz_joint3_sub,
            self.yz_joint4_sub,
            self.xz_joint1_sub,
            self.xz_joint2_sub,
            self.xz_joint3_sub,
            self.xz_joint4_sub] , 10)
        ts.registerCallback(callback)


def callback(self, data):
    yz_joint1 = data[0]
    yz_joint2 = data[1]
    yz_joint3 = data[2]
    yz_joint4 = data[3]
    xz_joint1 = data[4]
    xz_joint2 = data[5]
    xz_joint3 = data[6]
    xz_joint4 = data[7]
    
    # TODO calculate angles


# call the class
def main(args):
    v = vision()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)