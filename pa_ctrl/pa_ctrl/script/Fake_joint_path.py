#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, String, Bool
import std_msgs
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped, TransformStamped
import tf2_ros
import numpy as np
import tf2_geometry_msgs

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint 
from myworkcell_core.srv import *



class PublisherNode:
    def __init__(self):
        rospy.init_node('Fake_joint_oath_publisher')

        rospy.Subscriber('button',Bool, self.button_callback)


        init_pos = PoseStamped()
        init_pos.header.frame_id    =  "world"
        init_pos.pose.position.x    =  1.000
        init_pos.pose.position.y    =  0.001
        init_pos.pose.position.z    =  1.750

        init_pos.pose.orientation.x =  0.70711
        init_pos.pose.orientation.y = -4.33e-17
        init_pos.pose.orientation.z =  0.70711
        init_pos.pose.orientation.w = -4.33e-17

        self.compliant_pose = init_pos

        self.trajectory = JointTrajectory()
      

        # ROS Publisher for admittance position
        self.position_pub = rospy.Publisher('/compliant_position', PoseStamped, queue_size=10)

        # ROS publisher for commande
        self.trajectory_pub = rospy.Publisher("joint_path_command",JointTrajectory,queue_size=1)

        self.indx = 0

    def button_callback(self,data):
        self.position_pub.publish(self.compliant_pose)
        self.compliant_pose.pose.position.z += 0.050


def main():
    
    # Create an instance of the AdmittanceControlNode class
    publisher_node = PublisherNode()

    rospy.spin()
        
    

if __name__ == "__main__":
    
    main()
    
