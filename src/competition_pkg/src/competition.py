#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

TEAM_NAME = "TeamRed"
TEAM_PSW = "blla"

START_TIMER = "%s,%s,0,XR58" % (TEAM_NAME, TEAM_PSW)
END_TIMER = "%s,%s,-1,XR58" % (TEAM_NAME, TEAM_PSW)


if __name__=="__main__":
    
    rospy.init_node("competition", anonymous=True)
    
    license_pub = rospy.Publisher("license_plate", String, queue_size=10)
    movement_pub = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=10)

    license_pub.publish(START_TIMER)
    print(str(START_TIMER))

    movment_msg = Twist()
    movment_msg.linear.x = 1.0
    movement_pub.publish(movment_msg)
    
    time.sleep(5)

    license_pub.publish(END_TIMER)
    print(str(END_TIMER))

    rospy.spin()
