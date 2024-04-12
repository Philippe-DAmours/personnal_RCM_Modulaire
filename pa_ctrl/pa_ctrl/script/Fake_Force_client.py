#!/usr/bin/env python3
import socket
import rospy
import roslib
import rospkg
import sys
from std_msgs.msg import String

HOST = "192.168.1.100" # à vérifier
PORT = 11004


def force_client():
    pub = rospy.Publisher('Force_effector',String,queue_size=10)
    rospy.init_node('force_client',anonymous=True)
    rate = rospy.Rate(42) #42 hz
    timestep = 1/42.0
    time = 0

    #with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #    s.connect((HOST, PORT)) 
        
    while not rospy.is_shutdown():
        #data = s.recv(1024)
        #clean up data from bytes string to String
        #data = str(data)
        #data = data[2:]
        #data = data[:-3]
        #data = ' '.join(data.split())
        data = str(0.0) # fake force sensor value
        #Fe = [float(idx) for idx in data.split(' ')] # exemple to convert to list of float

        if 5*0.25 < time and time < 5*0.5 :
            data = str(10.0)
        else :
            data = str(0.0)
    
        # rospy.loginfo(data)
        
        pub.publish(data)
        time += timestep
        #pub.publish(Fe)
        rate.sleep()
        

        

if __name__ == "__main__":
    try:
        force_client()
    except rospy.ROSInterruptException:
        pass
