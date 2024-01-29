#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import moveit_commander


def initial_position():
    pub = rospy.Publisher('/initial_position',PoseStamped,queue_size=10)
    rospy.init_node('initial_position',anonymous=True)
    
    rate = rospy.Rate(42) #42 hz

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")



    data = group.get_current_pose()
    
        
    while not rospy.is_shutdown():
        #data = s.recv(1024)
        #clean up data from bytes string to String
        #data = str(data)
        #data = data[2:]
        #data = data[:-3]
        #data = ' '.join(data.split())
         
        


        rospy.loginfo(data)
        
        pub.publish(data)
        #pub.publish(Fe)
        rate.sleep()
        

        

if __name__ == "__main__":
    try:
        initial_position()
    except rospy.ROSInterruptException:
        pass
    rospy.init_node('force_client')

    initial_position()
