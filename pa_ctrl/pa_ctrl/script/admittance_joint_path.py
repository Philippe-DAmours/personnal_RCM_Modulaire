#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, String
import std_msgs
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped, TransformStamped
import tf2_ros
import numpy as np
import tf2_geometry_msgs

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint 
from myworkcell_core.srv import *



class AdmittanceControlNode:
    def __init__(self):
        rospy.init_node('admittance_control_node')

        # ROS Subscribers for initial position and force
        rospy.Subscriber('initial_position', PoseStamped, self.initial_position_callback)
        rospy.Subscriber('Force_effector', String, self.force_value_callback)

        # ROS tf listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Variables to store initial position and force
        self.xd_a  = 0.0
        self.xd_v  = 0.0
        self.xd_p  = 0.0

        self.initial_position = PoseStamped()
        self.compliant_pose   = PoseStamped()
        self.compliant_diff   = PoseStamped()
        self.compliant_vec    = Vector3Stamped()

        self.compliant_pose.header.frame_id = "world"
        self.compliant_diff.header.frame_id = "TCP"


        self.xc_a  = 0.0
        self.xc_v  = 0.0
        self.xc_p  = 0.0

        self.f_mes = 0.0
        self.f_des = 0.20

        #Caractérisation du système
        self.m  = 2.102 #kg
        self.kp = 1.0
        self.kd = 1.0

        self.rate = 42.0 #Hz
        self.ts = 1.0/self.rate
      

        # ROS Publisher for admittance position
        self.admittance_position_pub = rospy.Publisher('/compliant_position', PoseStamped, queue_size=10)


        ###

        # ROS Subscriber for compliante position
        # rospy.Subscriber('compliant_position',PoseStamped, self.admittanceCallback)

        # ROS publisher for commande
        self.trajectory_pub = rospy.Publisher("joint_path_command",JointTrajectory,queue_size=1)
        

        # ROS service client
        self.descartes_client = rospy.ServiceProxy("plan_path", PlanCartesianPath)

        

        # Initialise variable to initial command
        self.joint_command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.compliant_pose = PoseStamped()

        self.trajectory = JointTrajectory()
        self.trajectory.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        
        self.point = JointTrajectoryPoint()
        self.point.positions = self.joint_command
        self.trajectory.points.append(self.point)
        rospy.loginfo(self.trajectory)

        # rospy.sleep(1) # Sleeps for 1 sec
        # self.trajectory_pub.publish(self.trajectory)

        
        self.i = 0
        ###

    def initial_position_callback(self, data):
        # Callback function for initial position  
        self.initial_position = data
        # rospy.loginfo("callback force is calling back")
        
    def force_value_callback(self, data):
        # Callback function for force value
        self.f_mes = float(data.data)
        # AdmittanceControlNode.admittance_position()
        # rospy.loginfo("callback force is calling back")

    def admittance_position(self):
        # Example admittance control algorithm
        # Admittance Position = Initial Position + Force Value.
        self.xc_a = self.xd_a + 1.0/self.m*(self.f_mes - self.f_des - self.kd*(self.xc_v - self.xd_v) - self.kp*(self.xc_p - self.xd_p))
        self.xc_v = self.xc_v + self.xc_a*self.ts
        self.xc_p = self.xc_p + self.xc_v*self.ts

        self.compliant_vec.vector.z = self.xc_p
        
        #tf_transform = self.tf_buffer.lookup_transform("world","TCP",self.compliant_diff.header.stamp,rospy.Duration(1.0))
        tf_transform = TransformStamped()
        tf_transform.transform.rotation = self.initial_position.pose.orientation

        
        #pose_transform = tf2_geometry_msgs.do_transform_pose(self.compliant_diff,tf_transform)
        pose_transform = tf2_geometry_msgs.do_transform_vector3(self.compliant_vec,tf_transform)

        self.compliant_pose.pose.position.z = self.initial_position.pose.position.z + pose_transform.vector.z
        self.compliant_pose.pose.position.y = self.initial_position.pose.position.y + pose_transform.vector.y
        self.compliant_pose.pose.position.x = self.initial_position.pose.position.x + pose_transform.vector.x
        self.compliant_pose.pose.orientation = self.initial_position.pose.orientation

        # Publish the computed admittance position
        self.admittance_position_pub.publish(self.compliant_pose)
        # rospy.loginfo(self.compliant_pose)

        self.trajectory = JointTrajectory()
        self.trajectory.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.trajectory.points.append(self.point)
        self.trajectory.points[0].positions[0] = 0.0
        self.trajectory.points[0].positions[1] = 100*self.xc_p
        self.trajectory.points[0].positions[2] = 0.0
        self.trajectory.points[0].positions[3] = 0.0
        self.trajectory.points[0].positions[4] = 0.0
        self.trajectory.points[0].positions[5] = 0.0
        
        
        rospy.loginfo(self.trajectory)
        self.trajectory_pub.publish(self.trajectory)

def main():
    
    # Create an instance of the AdmittanceControlNode class
    admittance_node = AdmittanceControlNode()

    

    # Set the control rate (e.g., 10 Hz)
    rate = rospy.Rate(admittance_node.rate)
    
    while not rospy.is_shutdown():
        # Compute admittance position in the main loop
        admittance_node.admittance_position()
        # Sleep to maintain the control rate
        # rate.sleep()
        rate.sleep()
    
        

    
    

if __name__ == "__main__":
    
    main()
    
