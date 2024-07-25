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
        rospy.init_node('Fake_joint_path_publisher')

        rospy.Subscriber('button',Bool, self.button_callback)


        self.init_pos = PoseStamped()
        self.init_pos.header.frame_id    =  "world"
        self.init_pos.pose.position.x    =  1.240 #+ 0.050
        self.init_pos.pose.position.y    = -0.569
        self.init_pos.pose.position.z    =  0.940

        self.init_pos.pose.orientation.x =  0.70711
        #self.init_pos.pose.orientation.y = -4.33e-17
        self.init_pos.pose.orientation.z =  0.70711
        #self.init_pos.pose.orientation.w = -4.33e-17

        #self.init_pos.pose.orientation.x =  0.69
        self.init_pos.pose.orientation.y = -0.023
        #self.init_pos.pose.orientation.z =  0.711
        self.init_pos.pose.orientation.w = -0.0698

        self.compliant_pose = self.init_pos

        self.trajectory = JointTrajectory()
      

        # ROS Publisher for admittance position
        self.position_pub = rospy.Publisher('/compliant_position', PoseStamped, queue_size=10)

        # ROS publisher for commande
        self.trajectory_pub = rospy.Publisher("joint_path_command",JointTrajectory,queue_size=1)

        self.indx = 0

    def button_callback(self,data):
        #for x in range(20) :
        #    self.position_pub.publish(self.compliant_pose)
        #    self.compliant_pose.pose.position.z += 0.050
        #    rospy.sleep(0.300)
        #self.compliant_pose.pose.position.z =  0.50
        #self.compliant_pose.pose.position.x -=0.05
        #self.position_pub.publish(self.compliant_pose)
        #rospy.loginfo(self.compliant_pose)
        #rospy.sleep(0.300)
#
        #self.compliant_pose.pose.position.x +=0.05
        self.position_pub.publish(self.compliant_pose)
        rospy.loginfo(self.compliant_pose)
        rospy.sleep(0.300)

        self.compliant_pose.pose.position.z = 1.449
        #self.compliant_pose.pose.position.x =  1.0375 #+ 0.050
        self.position_pub.publish(self.compliant_pose)
        rospy.loginfo(self.compliant_pose)
        rospy.sleep(0.300)

        self.compliant_pose.pose.position.z = 1.51
        self.compliant_pose.pose.position.x = 1.20 #+ 0.050
        self.position_pub.publish(self.compliant_pose)
        rospy.loginfo(self.compliant_pose)
        rospy.sleep(0.300)

        self.compliant_pose.pose.position.z = 0.940
        self.compliant_pose.pose.position.x = 1.240  #+ 0.050


def main():
    
    # Create an instance of the AdmittanceControlNode class
    publisher_node = PublisherNode()

    rospy.spin()
        
    

if __name__ == "__main__":
    
    main()
    
