#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import moveit_commander
import tf 
import tf2_ros
import time
import yaml

def initial_position():
    pub = rospy.Publisher('/initial_position',PoseStamped,queue_size=10)
    rospy.init_node('initial_position',anonymous=True)
    
    
    rate = rospy.Rate(42) #42 hz

    # # Pour get la current position selon move_group
    # moveit_commander.roscpp_initialize(sys.argv)
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # group = moveit_commander.MoveGroupCommander("manipulator")
    # data = group.get_current_pose()

    # Pour get la current position selon tf


    #(trans,rot) = listener.lookupTransform("husky","world", rospy.Time(0))
    #tf_buffer = tf2_ros.Buffer()
    #tf_listener = tf2_ros.TransformListener(tf_buffer)
    #time.sleep(5.0)


    #tf_transform = tf_buffer.lookup_transform("world","TCP",rospy.Time(1.0))
    
    data = PoseStamped()
    data.pose.position.x    =  1.000
    data.pose.position.y    =  0.001
    data.pose.position.z    =  1.750

    data.pose.orientation.x =  0.70711
    data.pose.orientation.y = -4.33e-17
    data.pose.orientation.z =  0.70711
    data.pose.orientation.w = -4.33e-17


    #data.pose.position = trans
    #data.pose.orientation = rot
        
    while not rospy.is_shutdown():
        


        # rospy.loginfo(data)
        
        pub.publish(data)
        rate.sleep()
        

        

if __name__ == "__main__":
    try:
        initial_position()
    except rospy.ROSInterruptException:
        pass
