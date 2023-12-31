#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
import numpy as np

class AdmittanceControlNode:
    def __init__(self):
        rospy.init_node('admittance_control_node')

        # ROS Subscribers for initial position and force
        rospy.Subscriber('initial_position', PoseStamped, self.initial_position_callback)
        rospy.Subscriber('Force_effector', Float64, self.force_value_callback)

        # Variables to store initial position and force
        self.xd_a  = 0.0
        self.xd_v  = 0.0
        self.xd_p  = 0.0

        self.initial_position = PoseStamped()
        self.compliant_pose   = PoseStamped()


        self.xc_a  = 0.0
        self.xc_v  = 0.0
        self.xc_p  = 0.0

        self.f_mes = 0.0
        self.f_des = 0.20

        #Caractérisation du système
        self.m  = 2.102 #kg
        self.kp = 1.0
        self.kd = 1.0

        self.rate = 42 #Hz
        self.ts = 1/self.rate
      

        # ROS Publisher for admittance position
        self.admittance_position_pub = rospy.Publisher('compliant_position', PoseStamped, queue_size=10)

    def initial_position_callback(self, data):
        # Callback function for initial position
        self.initial_position = data
    def force_value_callback(self, data):
        # Callback function for force value
        self.f_mes = data.data

    def admittance_position(self):
        # Example admittance control algorithm
        # Admittance Position = Initial Position + Force Value.
        self.xc_a = self.xd_a + 1/self.m*(self.f_mes - self.f_des - self.kd*(self.xc_v - self.xd_v) - self.kp*(self.xc_p - self.xd_p))
        self.xc_v = self.xc_v + self.xc_a*self.ts
        self.xc_p = self.xc_p + self.xc_v*self.ts
        
        self.compliant_pose = self.initial_position
        self.compliant_pose.pose.position.z = self.initial_position.pose.position.z + self.xc_p


        # Publish the computed admittance position
        self.admittance_position_pub.publish(self.compliant_pose)
        rospy.loginfo(self.compliant_pose)

def main():
    try:
        # Create an instance of the AdmittanceControlNode class
        admittance_node = AdmittanceControlNode()

        # Set the control rate (e.g., 10 Hz)
        rate = rospy.Rate(admittance_node.rate)

        while not rospy.is_shutdown():
            # Compute admittance position in the main loop
            admittance_node.admittance_position()

            # Sleep to maintain the control rate
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
