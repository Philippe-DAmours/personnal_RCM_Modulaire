#!/usr/bin/env python3

import moveit.core
import rospy
from std_msgs.msg import Float64,Bool
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped, TransformStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PositionIKRequest, RobotState, JointConstraint
from moveit_msgs.srv import GetPositionIK
import tf2_ros
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from myworkcell_core.srv import PlanCartesianPath
import moveit_commander


class ComputeIKCallerNode:
    def __init__(self):
        # Cette node prend la position compliante d'admittance.py et 
        # appelle descarte_node pour avoir la transformé en joint
        # Envoi ensuite les commandes aux controlleurs (gazebo,réelle,etc.)
        
        rospy.init_node("descartes_caller_node")
        rospy.loginfo("initializing node")
        # ROS Subscriber for compliante position
        rospy.Subscriber('compliant_position',PoseStamped, self.admittanceCallback)

        ### ROS Subscriber for current joint_state
        #rospy.Subscriber("/joint_states",JointState,self.currentJointCallback)

        # ROS publisher for commande
        self.joint_0_pub = rospy.Publisher("MYROBOT/my_controller_joint_0/command",Float64,queue_size=1)
        self.joint_1_pub = rospy.Publisher("MYROBOT/my_controller_joint_1/command",Float64,queue_size=1)
        self.joint_2_pub = rospy.Publisher("MYROBOT/my_controller_joint_2/command",Float64,queue_size=1)
        self.joint_3_pub = rospy.Publisher("MYROBOT/my_controller_joint_3/command",Float64,queue_size=1)
        self.joint_4_pub = rospy.Publisher("MYROBOT/my_controller_joint_4/command",Float64,queue_size=1)
        self.joint_5_pub = rospy.Publisher("MYROBOT/my_controller_joint_5/command",Float64,queue_size=1)
        self.joint_6_pub = rospy.Publisher("MYROBOT/my_controller_joint_6/command",Float64,queue_size=1)

        ### ROS publisher for /joint_path_command for real robot
        self.joint_path_command_pub = rospy.Publisher("/joint_path_command",JointTrajectory,queue_size=1)

        ### ROS publisher for /last_joint_states, for the topic repeater to republish to /joint_states
        self.last_joint_state_pub = rospy.Publisher("/last_joint_states",JointState,queue_size=1)

        ### Subscriber for button when full traj
        rospy.Subscriber("button_full_traj",Bool,self.callback_publish_full_traj)

        # ROS service client
        self.descartes_client = rospy.ServiceProxy("plan_path", PlanCartesianPath)

        ### ROS serviec client pour move_group inverse Kinematic
        self.compute_ik_client = rospy.ServiceProxy("/compute_ik", GetPositionIK )

        

        # Initialise variable to initial command
        self.joint_command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.compliant_pose = PoseStamped()

        self.full_trajectory = JointTrajectory()

        self.joint_pose = RobotState()
        
        ### Dernière position de moveit
        self.current_joint_pose = JointState()
        ### Prenons à la place, juste la première position de moveit
        self.current_joint_pose = rospy.wait_for_message("/joint_states",JointState,timeout=3)

        

    def admittanceCallback(self,data):
        self.compliant_pose = data
        rospy.loginfo_once("callback is calling")
        self.computeIKSrvCall()
        
    #def currentJointCallback(self,data):
    #    self.current_joint_pose = data

    #def descarteSrvCall(self):
    #    req = self.compliant_pose.pose
    #    res = self.descartes_client(req)
    #    # self.publish_command(res)
    #    self.trajectory = res
    #    #rospy.loginfo(self.trajectory.trajectory.points[-1])
    #    self.publish_command()

    def computeIKSrvCall(self):
        req = PositionIKRequest()
        req.group_name = "manipulator"
        req.robot_state.joint_state = self.current_joint_pose
        req.ik_link_name = "TCP"
        req.pose_stamped = self.compliant_pose
        req.avoid_collisions = True
        
        #req.timeout = rospy.Duration(10)
        

        ### Ajoutons une joint constraint pour que l'axe du husky soit à zéro puisque la démo se fais sans cette axe.
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "husky_to_robot_base_plate"
        joint_constraint.position = 0.01
        joint_constraint.tolerance_below = 0.0
        joint_constraint.tolerance_above = 0.01
        joint_constraint.weight   = 100
        req.constraints.joint_constraints.append(joint_constraint)
        

        ### Ajoutons des contraintes autour des joints pour 
        ### évité des flip flop de solution
        pi = 3.1416
        
        #joint_constraint.joint_name = "joint_1"
        #joint_constraint.position = 0.0
        #joint_constraint.tolerance_below = pi/2
        #joint_constraint.tolerance_above = pi/2
        #req.constraints.joint_constraints.append(joint_constraint)

        #joint_constraint.joint_name = "joint_2"
        #joint_constraint.position = pi/4
        #joint_constraint.tolerance_below = pi/4
        #joint_constraint.tolerance_above = pi/4
        #req.constraints.joint_constraints.append(joint_constraint)

        joint_constraint3 = JointConstraint()
        joint_constraint3.weight = 1
        joint_constraint3.joint_name = "joint_3"
        joint_constraint3.position = 0.0
        joint_constraint3.tolerance_below = pi/2
        joint_constraint3.tolerance_above = pi/2
        req.constraints.joint_constraints.append(joint_constraint3)
        
        ### Configuration du joint 4 à gauche du 
        joint_constraint4 = JointConstraint()
        joint_constraint4.weight = 1
        joint_constraint4.joint_name = "joint_4"
        joint_constraint4.position = 0.0
        joint_constraint4.tolerance_below = pi/3
        joint_constraint4.tolerance_above = pi/3
        req.constraints.joint_constraints.append(joint_constraint4)

        #joint_constraint.joint_name = "joint_5"
        #joint_constraint.position = 0.0
        #joint_constraint.tolerance_below = pi/2
        #joint_constraint.tolerance_above = pi/2
        #req.constraints.joint_constraints.append(joint_constraint)


        rospy.loginfo(req)
        ### call le service
        res = self.compute_ik_client(req)
        ##rospy.loginfo(res.error_code.val)
        while not res.error_code.val == 1:
            rospy.loginfo("in loop while, spinning my life away")
            res = self.compute_ik_client(req)
        
        ### Reponse du service call
        self.joint_pose = res


        ### Mettre les joints dans un seul trajectory msg
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.time_from_start = rospy.Duration(0.3)
        trajectory_point.positions = self.joint_pose.solution.joint_state.position
        self.full_trajectory.header = self.joint_pose.solution.joint_state.header
        self.full_trajectory.joint_names = self.joint_pose.solution.joint_state.name
        self.full_trajectory.points.append(trajectory_point)

        ### update curent joint with last IK solution
        self.current_joint_pose.position = self.joint_pose.solution.joint_state.position

        rospy.loginfo(self.joint_pose)
        self.publish_command()

        
        
        


    def publish_command(self):
        
        # Transformons la trajectoir en une liste de position de joint publiable
        rospy.loginfo("Publishing command")
        single_point_traj = JointTrajectory()
        single_point_traj.header = self.current_joint_pose.header
        single_point_traj.joint_names = self.current_joint_pose.name
        single_point_traj.points.append(JointTrajectoryPoint())
        single_point_traj.points[0].positions = self.joint_pose.solution.joint_state.position
        
        #rospy.loginfo(single_point_traj)
        
        self.joint_path_command_pub.publish(single_point_traj)

    def callback_publish_full_traj(self,data):
        self.joint_path_command_pub.publish(self.full_trajectory)
        self.full_trajectory.points.clear()



def main():
    compute_ik_caller_node = ComputeIKCallerNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shuting down")



if __name__ == "__main__":
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        