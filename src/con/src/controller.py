#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty
import time

TEAM_NAME = "TeamRed"
TEAM_PSW = "blla"

START_TIMER = "%s,%s,0,XR58" % (TEAM_NAME, TEAM_PSW)
END_TIMER = "%s,%s,-1,XR58" % (TEAM_NAME, TEAM_PSW)


def getKey():
    """
    Function to get the keyboard input.
    """
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('moving_robot', anonymous=True)
    pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    vel_msg = Twist()

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w':
                vel_msg.linear.x = 0.4
            elif key == 's':
                vel_msg.linear.x = -0.4
            elif key == 'a':
                vel_msg.angular.z = 0.7
            elif key == 'd':
                vel_msg.angular.z = -0.7
            elif key == 'e':
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
            elif key == '\x03': # Ctrl+C
                break
            else:
                vel_msg.angular.z = 0
            
            pub.publish(vel_msg)
            time.sleep(0.2)
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            pub.publish(vel_msg)


    except KeyboardInterrupt:
        print("Shutting down")

    finally:
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        pub.publish(vel_msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)