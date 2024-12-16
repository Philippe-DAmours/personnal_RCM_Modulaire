#!/usr/bin/env python3

### rosrun pa_ctrl move_preset.py

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
import math
import numpy as np

class preset_move:
    def __init__(self) -> None:
        rospy.init_node("preset_move_node")

        self.traj_point = JointTrajectoryPoint()
        self.traj = JointTrajectory()
        self.joint_state = JointState()
        self.quicksave_joint_state = JointState()

        self.traj_pub = rospy.Publisher("/joint_path_command",JointTrajectory,queue_size=1)
        self.traj_point.time_from_start = rospy.Duration(0.3)
        self.traj.header.frame_id = "world"

        self.traj.joint_names = ["husky_to_robot_base_plate","joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]
        
        rospy.Subscriber("/console_move_pose_zero",Empty,self.consoleMovePoseZero)
        rospy.Subscriber("/console_move_pose_normal",Empty,self.consoleMovePoseNormal)
        rospy.Subscriber("/console_move_pose_travel",Empty,self.consoleMovePoseTravel)
        rospy.Subscriber("/console_move_pose_camera",Empty,self.consoleMovePoseCamera)
        rospy.Subscriber("/console_move_pose_camera_vertical",Empty,self.consoleMovePoseCameraVertical)
        rospy.Subscriber("/console_move_pose_1",Empty,self.consoleMovePosePreset1)
        rospy.Subscriber("/console_move_pose_2",Empty,self.consoleMovePosePreset2)
        rospy.Subscriber("/console_move_pose_3",Empty,self.consoleMovePosePreset3)
        rospy.Subscriber("/console_move_pose_4",Empty,self.consoleMovePosePreset4)
        rospy.Subscriber("/console_move_pose_quicksave",Empty,self.consoleMovePoseQuicksave)
        
        rospy.Subscriber("/console_quicksave_pose",Empty,self.consoleQuicksavePose)

        rospy.Subscriber("/joint_states",JointState,self.callbackJointPose)


        rospy.loginfo("preset node initialized")

    def appendAndPublish(self):
        rospy.loginfo("Moving to position")
        rospy.loginfo("degrees :" + str(self.traj_point.positions))
        self.traj_point.positions = np.radians(self.traj_point.positions)
        ### NOTE First joint is in mm. revert deg2rad
        self.traj_point.positions[0] = np.degrees(self.traj_point.positions[0])
        ### NOTE: FANUC transforme l'array de rad2deg
        ### Mettons le premiere joint prismatic en mm°/rad
        self.traj_point.positions[0] = np.radians(self.traj_point.positions[0])
        rospy.loginfo("radians :" + str(self.traj_point.positions))
        self.traj.points.append(self.traj_point)
        self.traj_pub.publish(self.traj)
        self.traj.points.clear()
        

    def consoleMovePoseZero(self,data):
        self.traj_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.appendAndPublish()

    def consoleMovePoseNormal(self,data):
        self.traj_point.positions = [510.50, 0.0, 25.0, -65.0, 0.0, 0.0, 0.0]
        self.appendAndPublish()

    def consoleMovePoseTravel(self,data):
        self.traj_point.positions = [0.0, 0.0, 0.0, -70.0, 0.0, -20.0, 0.0]
        self.appendAndPublish()

    def consoleMovePoseCamera(self,data):
        self.traj_point.positions = [500.0, 0.0, 0.0, -53.50, 0.0, -37.5, -45.0]
        self.appendAndPublish()

    def consoleMovePoseCameraVertical(self,data):
        self.traj_point.positions = [550.0, 0.0, 0.0, -53.50, -90.0, 90.0, 82.5]
        self.appendAndPublish()

    def consoleMovePosePreset1(self,data):
        self.traj_point.positions = [0.10, 0.0, 0.0, -53.50, 0.0, -37.5, -45.0]
        self.appendAndPublish()

    def consoleMovePosePreset2(self,data):
        ### Testons les presets de joints
        self.traj_point.positions = [10.0, 0.0, 25.0, -65.0, 0.0, 0.0, -30.64]
        self.appendAndPublish()

    def consoleMovePosePreset3(self,data):
        self.traj_point.positions = [0.0, 11.0, 56.33, -47.196, -19.19, 7.19, -30.64]
        self.appendAndPublish()

    def consoleMovePosePreset4(self,data):
        self.traj_point.positions = [0.0, 0.16389431059360504, 0.9975748062133789, -0.7767611742019653, -0.5247806310653687, 0.08886471390724182, -0.32888904213905334]
        self.traj_point.positions = np.degrees(self.traj_point.positions)
        self.appendAndPublish()

    def consoleMovePoseQuicksave(self,data):
        self.traj_point.positions = self.quicksave_joint_state.position
        self.appendAndPublish()

    def callbackJointPose(self,data):
        ### REceive position in radiant and with 7th axe at last
        self.joint_state = data

    def consoleQuicksavePose(self,data):
        self.quicksave_joint_state.position.append(self.joint_state.position[-1])
        self.quicksave_joint_state.position.extend(self.joint_state.position[0:6])
        self.quicksave_joint_state.position = np.degrees(self.quicksave_joint_state.position)
        self.quicksave_joint_state.position[0] = np.radians(self.quicksave_joint_state.position[0])
        rospy.loginfo("Position quicksaved at : \n" + str(self.quicksave_joint_state.position))
        

def main():
    preset_move_ = preset_move()
    rospy.spin()

if __name__ == "__main__":
    main()
    