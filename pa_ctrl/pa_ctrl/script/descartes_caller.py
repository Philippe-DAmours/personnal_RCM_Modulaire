#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped, TransformStamped
import tf2_ros
import numpy as np
from trajectory_msgs.msg import JointTrajectory
from myworkcell_core.srv import *

class DescartesCallerNode:
    def __init__(self):
        # Cette node prend la position compliante d'admittance.py et 
        # appelle descarte_node pour avoir la transformé en joint
        # Envoi ensuite les commandes aux controlleurs (gazebo,réelle,etc.)
        
        rospy.init_node("descartes_caller_node")
        rospy.loginfo("initializing node")
        # ROS Subscriber for compliante position
        rospy.Subscriber('compliant_position',PoseStamped, self.admittanceCallback)

        # ROS publisher for commande
        self.joint_0_pub = rospy.Publisher("MYROBOT/my_controller_joint_0/command",Float64,queue_size=1)
        self.joint_1_pub = rospy.Publisher("MYROBOT/my_controller_joint_1/command",Float64,queue_size=1)
        self.joint_2_pub = rospy.Publisher("MYROBOT/my_controller_joint_2/command",Float64,queue_size=1)
        self.joint_3_pub = rospy.Publisher("MYROBOT/my_controller_joint_3/command",Float64,queue_size=1)
        self.joint_4_pub = rospy.Publisher("MYROBOT/my_controller_joint_4/command",Float64,queue_size=1)
        self.joint_5_pub = rospy.Publisher("MYROBOT/my_controller_joint_5/command",Float64,queue_size=1)
        self.joint_6_pub = rospy.Publisher("MYROBOT/my_controller_joint_6/command",Float64,queue_size=1)


        # ROS service client
        self.descartes_client = rospy.ServiceProxy("plan_path", PlanCartesianPath)

        

        # Initialise variable to initial command
        self.joint_command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.compliant_pose = PoseStamped()

        self.trajectory = JointTrajectory()

        

    def admittanceCallback(self,data):
        self.compliant_pose = data
        rospy.loginfo_once("callback is calling")
        self.descarteSrvCall()
        

    def descarteSrvCall(self):
        req = self.compliant_pose.pose
        res = self.descartes_client(req)
        # self.publish_command(res)
        self.trajectory = res
        self.publish_command()

    def publish_command(self):
        
        # Transformons la trajectoir en une liste de position de joint publiable
        rospy.loginfo("Publishing command")
        self.joint_0_pub.publish(self.trajectory.points[1].positions[0])
        self.joint_1_pub.publish(self.trajectory.points[1].positions[1])
        self.joint_2_pub.publish(self.trajectory.points[1].positions[2])
        self.joint_3_pub.publish(self.trajectory.points[1].positions[3])
        self.joint_4_pub.publish(self.trajectory.points[1].positions[4])
        self.joint_5_pub.publish(self.trajectory.points[1].positions[5])
        self.joint_6_pub.publish(self.trajectory.points[1].positions[6])

def main():
    descartes_caller_node = DescartesCallerNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shuting down")



if __name__ == "__main__":
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        