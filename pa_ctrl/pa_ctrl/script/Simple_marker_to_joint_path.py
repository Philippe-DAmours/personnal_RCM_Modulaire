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
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker



class PublisherNode:
    def __init__(self):
        rospy.init_node('Fake_joint_path_publisher')
        rospy.loginfo("Simple marker to ik_caller started")

        ### Console command for user 
        rospy.Subscriber('console_marker_to_joint',Empty, self.console_callback)


        self.init_pos = PoseStamped()
        self.compliant_pose = self.init_pos


        # ROS Publisher for admittance position
        self.position_pub = rospy.Publisher('/compliant_position', PoseStamped, queue_size=10)

        ### ROS Subscriber for Marker from pa_uv
        self.marker_sub = rospy.Subscriber("/point_cloud_plane", Marker,self.callbackMarker, queue_size=10)


    def callbackMarker(self,data):
        ### Take the position of the marker
        marker = data
        self.compliant_pose.pose = marker.pose
        rospy.loginfo("pose updated")

    def console_callback(self,data):
        ### Publish the position for the compute_ik
        self.position_pub.publish(self.compliant_pose)



def main():
    
    # Create an instance of the PublisherNode class
    publisher_node = PublisherNode()

    rospy.spin()
        
    

if __name__ == "__main__":
    
    main()
    
