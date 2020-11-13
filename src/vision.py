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
        # initialize the node named vision
        rospy.init_node('vision', anonymous=True)

        self.v1_pub = rospy.Publisher(
            "/vec1", Float64MultiArray, queue_size=10)

        # image1 position topics
        self.yz_joint1_sub = message_filters.Subscriber("/estimates/yz/joint1", Float64MultiArray, queue_size=10)
        self.yz_joint2_sub = message_filters.Subscriber("/estimates/yz/joint2", Float64MultiArray, queue_size=10)
        self.yz_joint3_sub = message_filters.Subscriber("/estimates/yz/joint3", Float64MultiArray, queue_size=10)
        self.yz_joint4_sub = message_filters.Subscriber("/estimates/yz/joint4", Float64MultiArray, queue_size=10)

        # image2 position topics
        self.xz_joint1_sub = message_filters.Subscriber("/estimates/xz/joint1", Float64MultiArray, queue_size=10)
        self.xz_joint2_sub = message_filters.Subscriber("/estimates/xz/joint2", Float64MultiArray, queue_size=10)
        self.xz_joint3_sub = message_filters.Subscriber("/estimates/xz/joint3", Float64MultiArray, queue_size=10)
        self.xz_joint4_sub = message_filters.Subscriber("/estimates/xz/joint4", Float64MultiArray, queue_size=10)
        ts = message_filters.ApproximateTimeSynchronizer([
            self.yz_joint1_sub,
            self.yz_joint2_sub,
            self.yz_joint3_sub,
            self.yz_joint4_sub,
            self.xz_joint1_sub,
            self.xz_joint2_sub,
            self.xz_joint3_sub,
            self.xz_joint4_sub] , 9, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, d0, d1, d2, d3, d4, d5, d6, d7):
        yz_joint1 = d0.data
        yz_joint2 = d1.data
        yz_joint3 = d2.data
        yz_joint4 = d3.data
        xz_joint1 = d4.data
        xz_joint2 = d5.data
        xz_joint3 = d6.data
        xz_joint4 = d7.data
        
        # TODO calculate angles
    
        # get vectors for links
        vec_link1 = np.array([xz_joint2[0] - xz_joint1[0],
                            yz_joint1[0] - yz_joint2[0],
                            (xz_joint2[1] + yz_joint2[1])/2 - (xz_joint1[1] + yz_joint1[1])/2])
    
        vec_link2 = np.array([xz_joint3[0] - xz_joint2[0],
                            yz_joint2[0] - yz_joint3[0],
                            (xz_joint3[1] + yz_joint3[1])/2 - (xz_joint2[1] + yz_joint3[1])/2])
    
        vec_link3 = np.array([xz_joint4[0] - xz_joint3[0],
                            yz_joint3[0] - yz_joint4[0],
                            (xz_joint4[1] + yz_joint4[1])/2 - (xz_joint3[1] + yz_joint3[1])/2])
        
        a = Float64MultiArray()
        a.data = vec_link1
        self.v1_pub.publish(a)
        



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