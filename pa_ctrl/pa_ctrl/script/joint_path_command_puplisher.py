#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped, TransformStamped
import tf2_ros
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint 
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
        self.trajectory_pub = rospy.Publisher("/joint_path_command",JointTrajectory,queue_size=1)
        

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

        rospy.sleep(1) # Sleeps for 1 sec
        self.trajectory_pub.publish(self.trajectory)

        
        self.i = 0

        # Demo send command 3 position en joint
        #self.point = 


        

    def admittanceCallback(self,data):
        self.compliant_pose = data
        rospy.loginfo_once("callback is calling")
        self.descarteSrvCall()
        

    def descarteSrvCall(self):
        req = self.compliant_pose.pose
        res = self.descartes_client(req)
        
        ### Faisons premier test avec tout joint à zérp
        self.trajectory = JointTrajectory()
        self.trajectory.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.trajectory.points.append(self.point)
        self.trajectory.points[0].positions[0] = res.trajectory.points[-1].positions[1]
        self.trajectory.points[0].positions[1] = res.trajectory.points[-1].positions[2]
        self.trajectory.points[0].positions[2] = res.trajectory.points[-1].positions[3]
        self.trajectory.points[0].positions[3] = res.trajectory.points[-1].positions[4]
        self.trajectory.points[0].positions[4] = res.trajectory.points[-1].positions[5]
        self.trajectory.points[0].positions[5] = res.trajectory.points[-1].positions[6]
        
        self.trajectory.points.append(res.trajectory.points[-1])
        rospy.loginfo(self.trajectory)
        self.publish_command()


    def publish_command(self):
        
        # Transformons la trajectoir en une liste de position de joint publiable
        
        ### faison premier test avec tous les joints à zéro
        # self.joint_0_pub.publish(self.trajectory.trajectory.points[-1].positions[0])
        
        self.trajectory_pub.publish(self.trajectory)
        
        
    
        


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
        