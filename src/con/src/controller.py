#!/usr/bin/env python3

import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist

def getch():
    """
    Function to read a single character from the keyboard
    """
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    # Initialize the ROS node
    rospy.init_node('teleop_keyboard')

    # Create a publisher for the cmd_vel topic
    pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=10)

    # Set the publishing rate
    rate = rospy.Rate(10)

    # Set the initial velocities to zero
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0

    # Save the terminal settings
    settings = termios.tcgetattr(sys.stdin)

    # Main loop
    while not rospy.is_shutdown():
        # Read a single character from the keyboard
        key = getch()

        # Check if any of the movement keys are pressed
        if key == 'w':
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        elif key == 's':
            twist.linear.x = -0.2
            twist.angular.z = 0.0
        elif key == 'a':
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        elif key == 'd':
            twist.linear.x = 0.0
            twist.angular.z = -0.5
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish the twist message
        pub.publish(twist)

        # Sleep to maintain the publishing rate
        rate.sleep()

    # Restore the terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)