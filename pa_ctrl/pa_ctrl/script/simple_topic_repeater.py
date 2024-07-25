#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, String, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped, TransformStamped
import tf2_ros
import numpy as np
import tf2_geometry_msgs

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint 
from myworkcell_core.srv import *



class SimpleTopicRepeater:
    def __init__(self):
        rospy.init_node('simple_topic_repeater')

        rospy.Subscriber('/last_joint_states',JointState, self.update_message)


        self.message = JointState()
        self.ready = False
      

        # ROS publisher for commande
        self.trajectory_pub = rospy.Publisher("/joint_states",JointState,queue_size=1)


        self.rate = 100


    def update_message(self,data):
        self.message = data
        self.ready = True


def main():
    
    # Create an instance of the AdmittanceControlNode class
    simple_topic_repeater_node = SimpleTopicRepeater()

    

    # Set the control rate (e.g., 100 Hz)
    rate = rospy.Rate(simple_topic_repeater_node.rate)
    
    while not rospy.is_shutdown():
        if simple_topic_repeater_node.ready:
            rospy.loginfo(simple_topic_repeater_node.message)
            simple_topic_repeater_node.trajectory_pub.publish(simple_topic_repeater_node.message)
        rate.sleep()
        
    

if __name__ == "__main__":
    
    #try:
    #    main()
    #except rospy.ROSInterruptException:
    #    pass
    main()
