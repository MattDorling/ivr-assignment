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

        # initialize publishers for estimated angles
        self.ang1_pub = rospy.Publisher(
            "/estimates/angles/link_1", Float64, queue_size=10)
        self.ang2_pub = rospy.Publisher(
            "/estimates/angles/link_2", Float64, queue_size=10)
        self.ang3_pub = rospy.Publisher(
            "/estimates/angles/link_3", Float64, queue_size=10)
        self.ang4_pub = rospy.Publisher(
            "/estimates/angles/link_4", Float64, queue_size=10)


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
            self.xz_joint4_sub] , 9, 0.066, allow_headerless=True)
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

        yz_joint1 = np.array([399.0,532.0])
        xz_joint1 = yz_joint1

        # TODO calculate angles


        # vec = [x,y,z]    
        # get vectors for links
        vec_link1 = np.array([xz_joint2[0] - xz_joint1[0],
                            yz_joint2[0] - yz_joint1[0],
                            (xz_joint1[1] + yz_joint1[1])/2 - (xz_joint2[1] + yz_joint2[1])/2])

        vec_link2 = np.array([xz_joint3[0] - xz_joint2[0],
                            yz_joint3[0] - yz_joint2[0],
                            (xz_joint2[1] + yz_joint2[1])/2 - (xz_joint3[1] + yz_joint3[1])/2])

        vec_link3 = np.array([xz_joint4[0] - xz_joint3[0],
                            yz_joint4[0] - yz_joint3[0],
                            (xz_joint3[1] + yz_joint3[1])/2 - (xz_joint4[1] + yz_joint4[1])/2])
        vec_link2_xz = np.array([vec_link2[0], vec_link2[2]])
        vec_link2_yz = np.array([vec_link2[1], vec_link2[2]])

        a = Float64()
        a.data = self.angle( self.axis_vector(0,0,1, vec_link1), vec_link1)
        self.ang1_pub.publish(a)


 
        a.data = self.angle(self.axis_vector2(0,1,vec_link2_yz), vec_link2_yz)
        self.ang2_pub.publish(a)
        a.data = np.pi/2 - self.angle(self.axis_vector2(1,0,vec_link2_xz), vec_link2_xz)
        self.ang3_pub.publish(a)


        # a.data = self.angle(self.axis_vector(0,0,1, vec_link2), vec_link2)
        # self.ang2_pub.publish(a)
        # a.data = np.pi/2 - self.angle(self.axis_vector(1,0,0, vec_link2), vec_link2)
        # self.ang3_pub.publish(a)
        a.data = self.angle(vec_link3, vec_link2)
        self.ang4_pub.publish(a)

        # TODO improve the accuracy
        # I think I need to separate joint2 and joint3 - rotating around blue on x and y axes

    def angle(self, a, b):
        return np.arctan2(np.linalg.norm(np.cross(a,b)), np.dot(a,b))

    def axis_vector(self,x,y,z, vector):
        v = np.array([x,y,z])
        u = np.array([1,1,1])
        if vector[0] < 0:
            u[0] = -1
        if vector[1] < 0:
            u[1] = -1
        if vector[2] < 0:
            u[2] = -1
        return v * u

    def axis_vector2(self,a,b, vector):
        v = np.array([a,b])
        u = np.array([1,1])
        if vector[0] < 0:
            u[0] = -1
        if vector[1] < 0:
            u[1] = -1
        return v * u
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