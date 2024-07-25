#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Int16

def talker():
    pub = rospy.Publisher('nb_traj_topic', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    fake_int = 3
    
    pub.publish(fake_int)
    rospy.loginfo(fake_int)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass