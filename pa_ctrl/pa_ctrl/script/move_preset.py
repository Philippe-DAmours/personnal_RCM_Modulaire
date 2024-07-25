#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_msgs.msg import Empty
import math
import numpy as np

class preset_move:
    def __init__(self) -> None:
        rospy.init_node("preset_move_node")

        self.traj_point = JointTrajectoryPoint()
        self.traj = JointTrajectory()

        self.traj_pub = rospy.Publisher("/joint_path_command",JointTrajectory,queue_size=1)
        self.traj_point.time_from_start = rospy.Duration(0.3)
        self.traj.header.frame_id = "world"

        self.traj.joint_names = ["husky_to_robot_base_plate","joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]
        
        rospy.Subscriber("/console_move_pose_zero",Empty,self.consoleMovePoseZero)
        rospy.Subscriber("/console_move_pose_normal",Empty,self.consoleMovePoseNormal)
        rospy.Subscriber("/console_move_pose_travel",Empty,self.consoleMovePoseTravel)
        rospy.Subscriber("/console_move_pose_1",Empty,self.consoleMovePosePreset1)
        rospy.Subscriber("/console_move_pose_2",Empty,self.consoleMovePosePreset2)
        rospy.Subscriber("/console_move_pose_3",Empty,self.consoleMovePosePreset3)
        rospy.Subscriber("/console_move_pose_4",Empty,self.consoleMovePosePreset4)


        rospy.loginfo("preset node initialized")

    def appendAndPublish(self):
        rospy.loginfo("Moving to position")
        rospy.loginfo(self.traj_point.positions)
        self.traj_point.positions = np.radians(self.traj_point.positions)
        rospy.loginfo(self.traj_point.positions)
        self.traj.points.append(self.traj_point)
        self.traj_pub.publish(self.traj)
        self.traj.points.clear()
        

    def consoleMovePoseZero(self,data):
        self.traj_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.appendAndPublish()

    def consoleMovePoseNormal(self,data):
        self.traj_point.positions = [0.0, 0.0, 25.0, -65.0, 0.0, 0.0, 0.0]
        self.appendAndPublish()

    def consoleMovePoseTravel(self,data):
        pass

    def consoleMovePosePreset1(self,data):
        self.traj_point.positions = [0.01, 11.0, 56.33, -104.7, -19.19, 7.19, -30.64]
        self.appendAndPublish()

    def consoleMovePosePreset2(self,data):
        self.traj_point.positions = [0.0, 0.0, 25.0, -65.0, 0.0, 0.0, -30.64]
        self.appendAndPublish()

    def consoleMovePosePreset3(self,data):
        self.traj_point.positions = [0.0, 11.0, 56.33, -47.196, -19.19, 7.19, -30.64]
        self.appendAndPublish()

    def consoleMovePosePreset4(self,data):
        self.traj_point.positions = [0.0, 0.16389431059360504, 0.9975748062133789, -0.7767611742019653, -0.5247806310653687, 0.08886471390724182, -0.32888904213905334]
        self.traj_point.positions = np.degrees(self.traj_point.positions)
        self.appendAndPublish()

        

def main():
    preset_move_ = preset_move()
    rospy.spin()

if __name__ == "__main__":
    main()
    