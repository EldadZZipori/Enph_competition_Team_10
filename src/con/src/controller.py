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

def firstStrokeDrive(twist_msg):
    twist_msg.linear.x = 0.5
    pub.publish(twist_msg)
    rospy.sleep(0.8) # straight
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0.7
    pub.publish(twist_msg)
    rospy.sleep(3.2) # first left turn
    twist_msg.angular.z = 0
    twist_msg.linear.x = 0.5
    pub.publish(twist_msg)
    rospy.sleep(2.5) # first parking plate
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0.7
    pub.publish(twist_msg)
    rospy.sleep(2.8) # second left turn
    twist_msg.angular.z = 0
    twist_msg.linear.x = 0.5
    pub.publish(twist_msg)
    rospy.sleep(2)
    twist_msg.linear.x = 0
    pub.publish(twist_msg)


    return None


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    # print all model names
    """
    # Create a ROS client for the GetWorldProperties service
    rospy.wait_for_service("/gazebo/get_world_properties")
    get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)

    # Call the GetWorldProperties service to get the list of model names
    response = get_world_properties()
    model_names = response.model_names

    # Print the names of all the models in the world
    for name in model_names:
        print(name)
    """

    rospy.init_node('moving_robot', anonymous=True)
    pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    vel_msg = Twist()
    rospy.sleep(2)

    firstStrokeDrive(vel_msg)

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
            elif key == 'r':


                # Kill the existing instance of the node
                os.system("rosnode kill " + NODE_NAME)
            elif key == '\x03': # Ctrl+C
                break
            else:
                vel_msg.angular.z = 0
            
            pub.publish(vel_msg)
            rospy.sleep(0.2)
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