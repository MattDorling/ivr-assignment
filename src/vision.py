#!/usr/bin/env python3
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

        # initialize publisher for target position
        self.target_pub = rospy.Publisher(
            "/estimates/target", Float64MultiArray, queue_size=10)
        
        #initialize publisher for estimated end-effector position
        self.end_effector_pub = rospy.Publisher(
            "/estimates/end_effector", Float64MultiArray, queue_size=10)

        #initialize publisher for estimated end-effector position
        self.end_effector_pub = rospy.Publisher(
            "/estimates/end_effector", Float64MultiArray, queue_size=10)

        # image1 position topics
        self.yz_joint3_sub = message_filters.Subscriber(
            "/estimates/yz/joint3", Float64MultiArray, queue_size=10)
        self.yz_joint4_sub = message_filters.Subscriber(
            "/estimates/yz/joint4", Float64MultiArray, queue_size=10)
        self.yz_target_sub = message_filters.Subscriber(
            "/estimates/yz/target", Float64MultiArray, queue_size=10)

        # image2 position topics
        self.xz_joint3_sub = message_filters.Subscriber(
            "/estimates/xz/joint3", Float64MultiArray, queue_size=10)
        self.xz_joint4_sub = message_filters.Subscriber(
            "/estimates/xz/joint4", Float64MultiArray, queue_size=10)
        self.xz_target_sub = message_filters.Subscriber(
            "/estimates/xz/target", Float64MultiArray, queue_size=10)

        # synchronize joint positions for the callback
        ts = message_filters.ApproximateTimeSynchronizer([
            self.yz_joint3_sub,
            self.yz_joint4_sub,
            self.xz_joint3_sub,
            self.xz_joint4_sub,
            self.yz_target_sub,
            self.xz_target_sub] , 1, 0.05, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, d0, d1, d2, d3, t1, t2):
        yz_joint3 = d0.data
        yz_joint4 = d1.data
        xz_joint3 = d2.data
        xz_joint4 = d3.data
        yz_target = t1.data
        xz_target = t2.data

        # these joints do not move so they are hard-coded for now:
        yz_joint1 = np.array([400.0,532.0])
        xz_joint1 = yz_joint1
        yz_joint2 = np.array([400.0,477.0])
        xz_joint2 = yz_joint1

        # calculating the ratio to convert between pixels and meters:
        meters_per_pixel = self.pixel2meter(yz_joint1,yz_joint2,2.5)
        
        # defining axes
        x_axis = np.array([1,0,0])
        y_axis = np.array([0,1,0])
        z_axis = np.array([0,0,1])

        # changing coordinates so that the origin is the centre of yellow joint:
        yz_joint2  = np.array([yz_joint2[0] - yz_joint1[0], yz_joint1[1] - yz_joint2[1]])
        yz_joint3  = np.array([yz_joint3[0] - yz_joint1[0], yz_joint1[1] - yz_joint3[1]])
        yz_joint4  = np.array([yz_joint4[0] - yz_joint1[0], yz_joint1[1] - yz_joint4[1]])
        xz_joint2  = np.array([xz_joint2[0] - xz_joint1[0], xz_joint1[1] - xz_joint2[1]])
        xz_joint3  = np.array([xz_joint3[0] - xz_joint1[0], xz_joint1[1] - xz_joint3[1]])
        xz_joint4  = np.array([xz_joint4[0] - xz_joint1[0], xz_joint1[1] - xz_joint4[1]])
        # same for target:
        yz_target  = np.array([yz_target[0] - yz_joint1[0], yz_joint1[1] - yz_target[1]])
        xz_target  = np.array([xz_target[0] - xz_joint1[0], xz_joint1[1] - xz_target[1]])

        # get vector position of target:
        pos_target = np.array([ xz_target[0],
                                yz_target[0],
                               (xz_target[1] + yz_target[1])/2])

        # convert pixels to meter distance and publish:
        t = Float64MultiArray()
        bias = np.array([0.8,0.8,0.9])
        t.data = pos_target * meters_per_pixel * bias
        self.target_pub.publish(t)

        # get vector positions of the joints
        pos_joint2 = np.array([ xz_joint2[0],
                                yz_joint2[0],
                               (xz_joint2[1] + yz_joint2[1])/2])
        pos_joint3 = np.array([ xz_joint3[0],
                                yz_joint3[0],
                               (xz_joint3[1] + yz_joint3[1])/2])
        pos_joint4 = np.array([ xz_joint4[0],
                                yz_joint4[0],
                               (xz_joint4[1] + yz_joint4[1])/2])
        
        # convert pixels to meter distance for end_effector and publish:
        ef = Float64MultiArray()
        bias2 = np.array([0.8,-0.8,0.9])
        ef.data = pos_joint4 * meters_per_pixel * bias2
        self.end_effector_pub.publish(ef)

        # link vectors used to calculate angles
        vec_link2 = np.array(pos_joint3-pos_joint2)
        vec_link3 = np.array(pos_joint4-pos_joint3)

        # calculating the angles:
        theta_2 = self.angle(z_axis, vec_link2, x_axis)
        theta_3 = self.angle(z_axis, vec_link2, y_axis)
        r = np.matmul(self.rotation_matrix(x_axis, theta_2) , self.rotation_matrix(y_axis, theta_3))
        axis = np.matmul(r, x_axis)
        theta_4 = self.angle(vec_link2,vec_link3, axis)
        
        # publish angles:
        t_2 = Float64()
        t_3 = Float64()
        t_4 = Float64()
        t_2.data = theta_2
        t_3.data = theta_3
        t_4.data = theta_4
        self.ang2_pub.publish(t_2)
        self.ang3_pub.publish(t_3)
        self.ang4_pub.publish(t_4)

    # calculate the angle between vectors a and b, given a specified normal vector to the plane
    def angle(self, a, b, plane_norm):
        plane_norm = self.normalize(plane_norm)
        return np.arctan2(np.dot(np.cross(a,b),plane_norm),np.dot(a,b))

    # calculate the rotation matrix of theta about a given axis
    def rotation_matrix(self, axis, theta):
        axis = axis / np.sqrt(np.dot(axis, axis))
        a = np.cos(theta / 2.0)
        b, c, d = -axis * np.sin(theta / 2.0)
        return np.array([[a*a + b*b - c*c - d*d, 2 * (b*c + a*d), 2 * (b*d - a*c)],
                         [2 * (b*c - a*d), a*a + c*c - b*b - d*d, 2 * (c*d + a*b)],
                         [2 * (b*d + a*c), 2 * (c*d - a*b), a*a + d*d - b*b - c*c]])

    # normalize a vector
    def normalize(self, v):
        return v / np.linalg.norm(v)

    # calculating the ratio to convert between pixels and meters:
    def pixel2meter(self, pos_1, pos_2, meters):
        dist = np.sum((pos_1 - pos_2)**2)
        return float(meters / np.sqrt(dist))

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
