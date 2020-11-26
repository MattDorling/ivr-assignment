#!/usr/bin/env python3
import sys
import rospy
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class control:
    
    def __init__(self):
        # initialize the node named control
        rospy.init_node('control', anonymous=True)
        
        #Get angles from joint_states topic
        self.ang1_sub = message_filters.Subscriber(
            "/robot/joint_states", JointState, queue_size=10)
        self.ang2_sub = message_filters.Subscriber(
            "/robot/joint_states", JointState, queue_size=10)
        self.ang3_sub = message_filters.Subscriber(
            "/robot/joint_states", JointState, queue_size=10)
        self.ang4_sub = message_filters.Subscriber(
            "/robot/joint_states", JointState, queue_size=10)

        #Get target position published from vision
        self.target_sub = message_filters.Subscriber(
            "/estimates/target", Float64MultiArray, queue_size=10)
        
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
        
        # initialize a publisher to send forward kinematics to topic for comparing
        self.forward_kinematics_x_pub = rospy.Publisher(
            "/estimates/forward_kinematics_x", Float64, queue_size=10)
        self.forward_kinematics_y_pub = rospy.Publisher(
            "/estimates/forward_kinematics_y", Float64, queue_size=10)
        self.forward_kinematics_z_pub = rospy.Publisher(
            "/estimates/forward_kinematics_z", Float64, queue_size=10)
        
        # initialize errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')     
        # initialize error and derivative of error for trajectory tracking  
        self.error = np.array([0.0,0.0,0.0], dtype='float64')  
        self.error_d = np.array([0.0,0.0,0.0], dtype='float64')
        
        # synchronize callback arguments
        ts = message_filters.ApproximateTimeSynchronizer([
            self.ang1_sub,
            self.ang2_sub,
            self.ang3_sub,
            self.ang4_sub,
            self.target_sub] , 5, 0.066, allow_headerless=True)
        ts.registerCallback(self.callback)
        
        
    def forward_kinematics(self, t1, t2, t3, t4):
        
        # Add degrees to theta1 & theta2, with the angles between xi-1 & xi along zi-1
        # in DH parameters
        t1 = t1 + np.pi/2
        t2 = t2 + np.pi/2
        
        # Plot given angles into given forward kinematics equation
        end_effector = np.array([[3*(np.sin(t1)*np.sin(t3) + np.cos(t1)*np.cos(t2)*np.cos(t3))*np.cos(t4) + 3.5*np.sin(t1)*np.sin(t3) - 3*np.sin(t2)*np.sin(t4)*np.cos(t1) + 3.5*np.cos(t1)*np.cos(t2)*np.cos(t3)], 
                                 [3*(np.sin(t1)*np.cos(t2)*np.cos(t3) + np.sin(t3)*np.cos(t1))*np.cos(t4) - 3*np.sin(t1)*np.sin(t2)*np.sin(t4) + 3.5*np.sin(t1)*np.cos(t2)*np.cos(t3) + 3.5*np.sin(t3)*np.cos(t1)],
                                 [3*np.sin(t2)*np.cos(t3)*np.cos(t4) + 3.5*np.sin(t2)*np.cos(t3) + 3*np.sin(t4)*np.cos(t2) + 2.5]])
        return end_effector
    
    
    def calculate_jacobian(self, t1, t2, t3, t4):

        # Add degrees to theta1 & theta2, with the angles between xi-1 & xi along zi-1
        # in DH parameters
        t1 = t1 + np.pi/2
        t2 = t2 + np.pi/2
  
        # Plot given angles into given jacobian equation
        jacobian = np.array([[(-3*np.sin(t1)*np.cos(t2)*np.cos(t3) + 3*np.sin(t3)*np.cos(t1))*np.cos(t4) + 3*np.sin(t1)*np.sin(t2)*np.sin(t4) - 3.5*np.sin(t1)*np.cos(t2)*np.cos(t3) + 3.5*np.sin(t3)*np.cos(t1),
                              -3*np.sin(t2)*np.cos(t1)*np.cos(t3)*np.cos(t4) - 3.5*np.sin(t2)*np.cos(t1)*np.cos(t3) - 3*np.sin(t4)*np.cos(t1)*np.cos(t2),
                              (3*np.sin(t1)*np.cos(t3) - 3*np.sin(t3)*np.cos(t1)*np.cos(t2))*np.cos(t4) + 3.5*np.sin(t1)*np.cos(t3) - 3.5*np.sin(t3)*np.cos(t1)*np.cos(t2),
                              -(3*np.sin(t1)*np.sin(t3) + 3*np.cos(t1)*np.cos(t2)*np.cos(t3))*np.sin(t4) - 3*np.sin(t2)*np.cos(t1)*np.cos(t4)],
                             [(-3*np.sin(t1)*np.sin(t3) + 3*np.cos(t1)*np.cos(t2)*np.cos(t3))*np.cos(t4) - 3.5*np.sin(t1)*np.sin(t3) - 3*np.sin(t2)*np.sin(t4)*np.cos(t1) + 3.5*np.cos(t1)*np.cos(t2)*np.cos(t3),
                              -3*np.sin(t1)*np.sin(t2)*np.cos(t3)*np.cos(t4) - 3.5*np.sin(t1)*np.sin(t2)*np.cos(t3) - 3*np.sin(t1)*np.sin(t4)*np.cos(t2),
                              (-3*np.sin(t1)*np.sin(t3)*np.cos(t2) + 3*np.cos(t1)*np.cos(t3))*np.cos(t4) - 3.5*np.sin(t1)*np.sin(t3)*np.cos(t2) + 3.5*np.cos(t1)*np.cos(t3),
                              -(3*np.sin(t1)*np.cos(t2)*np.cos(t3) + 3*np.sin(t3)*np.cos(t1))*np.sin(t4) - 3*np.sin(t1)*np.sin(t2)*np.cos(t4)],
                             [0,
                              -3*np.sin(t2)*np.sin(t4) + 3*np.cos(t2)*np.cos(t3)*np.cos(t4) + 3.5*np.cos(t2)*np.cos(t3),
                              -3*np.sin(t2)*np.sin(t3)*np.cos(t4) - 3.5*np.sin(t2)*np.sin(t3),
                              -3*np.sin(t2)*np.sin(t4)*np.cos(t3) + 3*np.cos(t2)*np.cos(t4)]])
    
        return jacobian
    
    
    def control_closed(self, t1, t2, t3, t4, tar_pos):
        # P gain
        K_p = np.array([[7,0,0],[0,7,0],[0,0,7]])
        # D gain
        K_d = np.array([[0.05,0,0],[0,0.05,0],[0,0,0.05]])
        # estimate time step
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time
        # robot end-effector position
        posfk = self.forward_kinematics(t1, t2, t3, t4)
        pos = np.array([posfk[0][0], posfk[1][0], posfk[2][0]])
        # desired target
        pos_d= np.array([tar_pos[0], tar_pos[1], tar_pos[2]])
        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error)/dt
        # estimate error
        self.error = pos_d-pos
        q = np.array([0, t2, t3, t4]) # estimate initial value of joints'
        J_inv = np.linalg.pinv(self.calculate_jacobian(t1, t2, t3, t4))  # calculating the psudeo inverse of Jacobian
        #J_inv = np.linalg.pinv(np.array([[1,1,1,1],[1,1,1,1],[1,1,1,1]]))
        dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
        q_d = q + (dt * dq_d)  # control input (angular position of joints)
        return q_d
    
    def callback(self, t1, t2, t3, t4, tar_pos):
        
        # Define joint angles
        theta1 = t1.position[0]
        theta2 = t2.position[1]
        theta3 = t3.position[2]
        theta4 = t4.position[3]

        target_position = tar_pos.data
        
        # Calculate forward kinematics separately to publish
        fk = self.forward_kinematics(theta1, theta2, theta3, theta4)
        
        # send control commands to joints
        q_d = self.control_closed(theta1, theta2, theta3, theta4, target_position)
        self.joint1=Float64()
        self.joint1.data= q_d[0]
        self.joint2=Float64()
        self.joint2.data= q_d[1]
        self.joint3=Float64()
        self.joint3.data= q_d[2]
        self.joint4=Float64()
        self.joint4.data= q_d[3]

    
        # Publish the results
        try:
            self.robot_joint1_pub.publish(0)
            self.robot_joint2_pub.publish(self.joint2)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)
            self.forward_kinematics_x_pub.publish(fk[0][0])
            self.forward_kinematics_y_pub.publish(fk[1][0])
            self.forward_kinematics_z_pub.publish(fk[2][0])
        
        except CvBridgeError as e:
            print(e)
        
    
# call the class
def main(args):
    
    c = control()
    #c.callback()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
  
# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
