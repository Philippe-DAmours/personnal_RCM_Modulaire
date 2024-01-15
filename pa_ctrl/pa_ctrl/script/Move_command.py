#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
import moveit_commander

class MoveCommandNode():
    def __init__(self):
        rospy.init_node("move_command_node")

        rospy.Subscriber('compliant_position',PoseStamped,self.compliant_position_callback)
        self.group = moveit_commander.MoveGroupCommander("manipulator")

        self.rate = 42/100 #z
        self.condition = True
        self.pose = Pose()

    def compliant_position_callback(self, data):
        self.pose = data.pose
        #rospy.loginfo(data.pose)
        

    def go(self):
        self.group.set_pose_target(self.pose)
        rospy.loginfo(self.pose)
        self.group.go(wait=True)
        
def main():
    try:
        move_command_node = MoveCommandNode()

        rate = rospy.Rate(move_command_node.rate)

        while not rospy.is_shutdown():
            rate.sleep()
            move_command_node.go()

        

    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
        