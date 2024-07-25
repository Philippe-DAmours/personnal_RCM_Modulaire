#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty

class Marker_pub_node:
    def __init__(self):
        rospy.init_node('Marker_pub_node')
        rospy.loginfo("noke Marker_pub started")

        rospy.Subscriber('/console_new_marker',Empty, self.consoleCallback)
        self.marker_pub = rospy.Publisher("/point_cloud_plane", Marker,queue_size=1)
        
        ### Fake arbitary marker
        self.marker = Marker()
        self.marker.header.frame_id    = "world"
        self.marker.pose.position.x    =  1.240
        self.marker.pose.position.y    = -0.569
        self.marker.pose.position.z    =  0.940
        self.marker.pose.orientation.x =  0.70711
        self.marker.pose.orientation.y = -0.023
        self.marker.pose.orientation.z =  0.70711
        self.marker.pose.orientation.w = -0.0698


        ### Arbitary valid posittion from Fake_initial_position for starter --------------
        ##data.header.frame_id    =  "world"
        ##data.pose.position.x    =  1.240
        ##data.pose.position.y    = -0.569
        ##data.pose.position.z    =  0.940
    
        ##data.pose.orientation.x =  0.70711
        ##data.pose.orientation.y = -0.023
        ##data.pose.orientation.z =  0.70711
        ##data.pose.orientation.w = -0.0698
        ### ------------------------------------------------------------------------------


    def consoleCallback(self,data):
        self.marker_pub.publish(self.marker)    
    

def main():
    
    marker_pub_node_ = Marker_pub_node()

    rospy.spin()
    
    

if __name__ == '__main__':
    main()