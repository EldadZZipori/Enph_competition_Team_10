#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

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

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w':
                vel_msg = Twist()
                vel_msg.linear.x = 0.5
                pub.publish(vel_msg)
            elif key == 's':
                vel_msg = Twist()
                vel_msg.linear.x = -0.5
                pub.publish(vel_msg)
            elif key == 'a':
                vel_msg = Twist()
                vel_msg.angular.z = 0.7
                pub.publish(vel_msg)
            elif key == 'd':
                vel_msg = Twist()
                vel_msg.angular.z = -0.7
                pub.publish(vel_msg)
            elif key == '\x03': # Ctrl+C
                break

            rate.sleep()

    except KeyboardInterrupt:
        print("Shutting down")

    finally:
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        pub.publish(vel_msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)