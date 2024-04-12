#!/usr/bin/env python3
import socket
import rospy
import roslib
import rospkg
import sys
from std_msgs.msg import Int16


HOST = "192.168.1.100" # à vérifier
PORT = 11006

def service_callback(data):
    rate = rospy.Rate(100)
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.sendall(data.data.to_bytes(16,byteorder='big'))
            
            # rospy.spin()
            rospy.sleep(0.1)
            s.close()
            
            #while not rospy.is_shutdown():
            #    rospy.spin()
            #retour = s.recv(1024)
            #rospy.loginfo(str(retour,encoding='utf-8'))
           


    except ConnectionRefusedError:
        print("Connection to", HOST, "on port", PORT, "refused.")
            

            
def main():
    rospy.init_node('status_client', anonymous=True)
    rospy.loginfo("Starting node : Status_client")
    rospy.Subscriber("nb_traj_topic",Int16,service_callback)
    rospy.spin()

                



if __name__ == "__main__":
    main()

