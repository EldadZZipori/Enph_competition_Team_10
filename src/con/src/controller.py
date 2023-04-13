#!/usr/bin/env python3

import rospy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty
import time
import math

TEAM_NAME = "Team11"
TEAM_PSW = "JessEldad"

START_TIMER = "%s,%s,0,XR58" % (TEAM_NAME, TEAM_PSW)
END_TIMER = "%s,%s,-1,XR58" % (TEAM_NAME, TEAM_PSW)

PID_DRIVE_FLAG = False
RED_LINE_FLAG = False
DEBUG = False
ENTER_CENTER = False
PEDESTRIAN_TARGET = 2

# Initialize the error variables
last_error = 0
integral_error = 0

# Initialize the PID constants
kp = 0.01
ki = 0.0001
kd = 0.0001

# Define the target distance from the right line (in pixels)
target_distance = 380

# Define the center line of the image
center_line = 1280 / 2

y_location = 600
x_axis_len = 1280
y_axis_len= 800

distance_from_red_line = 400

previous_x = -1

previous_enter_inner_detection = []
cross_wait = []

pedestrians_seen = 0

twist_msg = Twist()



def getKey():
    """
    Function to get the keyboard input.
    """
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def firstStrokeDrive():
    global twist_msg

    twist_msg.linear.x = 0.5
    mov_pub.publish(twist_msg)
    rospy.sleep(0.75) # straight
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0.7
    mov_pub.publish(twist_msg)
    rospy.sleep(3.1) # first left turn

    """
    twist_msg.angular.z = 0
    twist_msg.linear.x = 0.5
    mov_pub.publish(twist_msg)
    rospy.sleep(2.5) # first parking plate
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0.7
    mov_pub.publish(twist_msg)
    rospy.sleep(2.8) # second left turn
    twist_msg.angular.z = 0
    twist_msg.linear.x = 0.5
    mov_pub.publish(twist_msg)
    rospy.sleep(2)
    """
    twist_msg.angular.z = 0
    twist_msg.linear.x = 0
    mov_pub.publish(twist_msg)


    return None

def hill_hard_code():
    global twist_msg, PID_DRIVE_FLAG

    twist_msg.linear.x = 0.5
    mov_pub.publish(twist_msg)
    rospy.sleep(0.5) # straight
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0.82
    mov_pub.publish(twist_msg)
    rospy.sleep(2.52) # first left turn
    twist_msg.angular.z = 0
    twist_msg.linear.x = 0.5
    mov_pub.publish(twist_msg)
    rospy.sleep(4.9) # first parking plate
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0.8
    mov_pub.publish(twist_msg)
    rospy.sleep(2.6) # first left turn
    twist_msg.angular.z = 0
    twist_msg.linear.x = 0.5
    mov_pub.publish(twist_msg)
    rospy.sleep(1) # first parking plate
    twist_msg.angular.z = 0
    twist_msg.linear.x = 0
    mov_pub.publish(twist_msg)

    PID_DRIVE_FLAG = True


def image_callback(image):
    global last_error, integral_error, PID_DRIVE_FLAG, twist_msg, x_axis_len, y_axis_len, pedestrians_seen, ENTER_CENTER, previous_enter_inner_detection, cross_wait
    
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
        print(e)

    shape = cv_image.shape
       
    #x_axis_len = shape[1]
    #y_axis_len = shape[0]

    """
    # Convert the image to grayscale
    gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

    # Threshold the image to obtain a binary image of white pixels
    ret, thresh = cv.threshold(gray, 150, 255, cv.THRESH_BINARY)

    # Apply a bitwise AND operation to obtain only the white pixels
    white = cv.bitwise_and(cv_image, cv_image, mask=thresh)
    """
    white = detection_on_grass(cv_image)
    first_white_location_right = detect_right_line(white)
    first_white_location_left = detect_left_line(white)
    error = 0 

    
    if not first_white_location_right == -1:
        distance_from_right = first_white_location_right - center_line
        error = -1 * (distance_from_right - target_distance) # needs to be negative 
    if first_white_location_right == -1: # change to depend on how many pedestrains are detected
        
        if not first_white_location_left == -1:
            distance_from_left = center_line - first_white_location_left
            error = (distance_from_left  - target_distance) # needs to be positive
        else:
            error = last_error
    
    first_blue_location_left = detect_blue(cv_image)
    
    if ENTER_CENTER and first_white_location_left == -1 and first_blue_location_left == -1: 
        previous_enter_inner_detection.append(1)
        if len(previous_enter_inner_detection) ==  30:
            PID_DRIVE_FLAG = False
            #enter_inner()
            #PID_DRIVE_FLAG = True
            
    else:
        previous_enter_inner_detection = []

    """
    PID control
    """
    if DEBUG:
        cv.imshow("", cv_image)
        cv.waitKey(3)

    # Compute the PID control output
    integral_error += error
    derivative_error = error - last_error
    control_output = kp * error #+ ki * integral_error + kd * derivative_error
    #print(control_output)

    pedestrian_detection(cv_image)

    # Publish the control output as a twist message
    if PID_DRIVE_FLAG: 
        cross_wait.append(1)
        twist_msg.linear.x = 0.2
        twist_msg.angular.z = control_output
        mov_pub.publish(twist_msg)

    # Store the current error for the next iteration
    last_error = error
    return None

def pedestrian_detection(cv_image):
    global PID_DRIVE_FLAG, RED_LINE_FLAG, previous_x, cross_wait

    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
    kernel = np.ones((5,5), np.uint8)

    if PID_DRIVE_FLAG: # detect the red line

        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])

        red_mask = cv.inRange(hsv, lower_red, upper_red)

        red_mask = cv.erode(red_mask, kernel, iterations=1)
        red_mask = cv.dilate(red_mask, kernel, iterations=1)

        contours, hierarchy = cv.findContours(red_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(cv_image, contours, -1, (0, 0, 255), 2)

        if not len(contours) == 0:
            for cnt in contours:
                # Fit a line to the contour
                [vx, vy, x, y] = cv.fitLine(cnt, cv.DIST_L2, 0, 0.01, 0.01)
                slope = vy / vx
                y_intercept = y - slope * x

                # Calculate the distance from the line to a point (x,y)
                x = 100 # Replace with the x coordinate of the point you want to calculate the distance to the line
                y = 200 # Replace with the y coordinate of the point you want to calculate the distance to the line
                distance = int(abs(slope * x - y + y_intercept) / math.sqrt(slope**2 + 1))
                
                #print(len(cross_wait))
                if distance >= distance_from_red_line - 20 and distance <= distance_from_red_line + 20 and len(cross_wait) > 100: 
                    #print("found")
                    twist_msg.linear.x = 0
                    twist_msg.angular.z = 0
                    mov_pub.publish(twist_msg)
                    PID_DRIVE_FLAG = False
                    RED_LINE_FLAG = True
                    return None

    elif RED_LINE_FLAG:
        #print("pedestrian")
        # Load the pedestrian detector
        hog = cv.HOGDescriptor()
        hog.setSVMDetector(cv.HOGDescriptor_getDefaultPeopleDetector())

        height = cv_image.shape[0]

        # Define the region of interest
        roi = cv_image[height-390:height, :]

        # Detect pedestrians in the image
        pedestrians, weights = hog.detectMultiScale(roi, winStride=(8, 8), padding=(32, 32), scale=1.05, groupThreshold=2)

        # Draw bounding boxes around the detected pedestrians
        for (x, y, w, h) in pedestrians:
            # Check if the detected object is too tall to be a pedestrian
            if h/w > 3:
                continue
            # cv.rectangle(roi, (x, y), (x + w, y + h), (0, 0, 255), 2)

            if previous_x == -1:
                previous_x = x
            elif previous_x < 600:
                if x > 780: # continue driving 
                    cross()
            else:
                if x < 515: #continue driving 
                    cross()
        rospy.sleep(0.1)

    return None

def cross():
    global previous_x,twist_msg, pedestrians_seen, PID_DRIVE_FLAG, RED_LINE_FLAG,cross_wait
    cross_wait = []
    previous_x = -1
    RED_LINE_FLAG = False

    twist_msg.linear.x = 0.8
    mov_pub.publish(twist_msg)
    rospy.sleep(0.9)
    twist_msg.linear.x = 0
    mov_pub.publish(twist_msg)
    pedestrians_seen += 1

    if not pedestrians_seen == 2:
        PID_DRIVE_FLAG = True
        rospy.sleep(1)
    else:
        pedestrians_seen = 0
        hill_hard_code()

    return None

def enter_inner():
    global twist_msg

    twist_msg.angular.z = 0
    twist_msg.linear.x = 0
    mov_pub.publish(twist_msg)
    twist_msg.angular.z = 0.7
    mov_pub.publish(twist_msg)
    rospy.sleep(3.2) # first left turn

def detection_on_grass(cv_image):
    global twist_msg, DEBUG
    # Convert the image to HSV color space
    gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

    # Apply a threshold to make white colors stronger
    thresh_value = 175
    _, thresh = cv.threshold(gray, thresh_value, 255, cv.THRESH_BINARY)

    # Apply a contrast adjustment
    alpha = 2.2  # Contrast control (1.0-3.0)
    beta = 0    # Brightness control (0-100)
    adjusted_img = cv.convertScaleAbs(thresh, alpha=alpha, beta=beta)

    # Display the result
    #if DEBUG and not pedestrians_seen == PEDESTRIAN_TARGET:
        #cv.imshow('White Lines Detection', adjusted_img)
        #cv.waitKey(3)

    return adjusted_img

def detect_blue(cv_image):
    global pedestrians_seen, DEBUG, x_axis_len, y_axis_len
    # Convert the image to the HSV color space
    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

    # Define the lower and upper bounds of the blue color in HSV
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # Threshold the image to extract blue colors
    mask = cv.inRange(hsv, lower_blue, upper_blue)

    # Apply the mask to the original image
    result = cv.bitwise_and(cv_image, cv_image, mask=mask)
    first_blue_location_left = -1

    for index in reversed(range(int(x_axis_len / 2) - int(target_distance + 100), int(x_axis_len / 2))):
        color = result[y_location][index]
        R, G, B = color
        if R >= 110 and G >= 10 and B >= 10:
        #if color >= 1:
            first_blue_location_left = index
            index = y_axis_len + 1

    if DEBUG:
        cv.imshow('Filtered Image', result)
        cv.waitKey(3)

    return first_blue_location_left


def detect_left_line(white):
    first_white_location_left = -1
    for index in reversed(range(int(x_axis_len / 2) - int(target_distance), int(x_axis_len / 2))):
        color = white[y_location][index]
        #R, G, B = color
        #if R >= 245 and G >= 245 and B >= 245:
        if color >= 1:
            first_white_location_left = index
            index = y_axis_len + 1
    return first_white_location_left

def detect_right_line(white):
    first_white_location_right = -1
    for index in range(int(x_axis_len / 2) ,int(x_axis_len / 2) + int(target_distance)):
        color = white[y_location][index]
        #R, G, B = color
        if color >= 1:
            first_white_location_right = index
            index = y_axis_len + 1
    return first_white_location_right

"""
Init Rospy
"""
rospy.init_node('moving_robot', anonymous=True)
license_pub = rospy.Publisher("license_plate", String, queue_size=10)
mov_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=10)
image_sub = rospy.Subscriber('/R1/pi_camera/image_raw',Image, image_callback)
cv_bridge = CvBridge()
rate = rospy.Rate(10) # 10hz
rospy.sleep(2)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    if DEBUG:
        try:
            while not rospy.is_shutdown():
                key = getKey()
                if key == 'w':
                    twist_msg.linear.x = 0.4
                elif key == 's':
                    twist_msg.linear.x = -0.4
                elif key == 'a':
                    twist_msg.angular.z = 0.7
                elif key == 'd':
                    twist_msg.angular.z = -0.7
                elif key == 'e':
                    twist_msg.linear.x = 0
                    twist_msg.angular.z = 0
                elif key == 'p':
                    PID_DRIVE_FLAG = not PID_DRIVE_FLAG
                elif key == 'f':
                    firstStrokeDrive()
                elif key == 'i':
                    ENTER_CENTER = not ENTER_CENTER
                    print("[*] Inner loop", ENTER_CENTER)
                elif key == 'c':
                    RED_LINE_FLAG = not RED_LINE_FLAG
                    print("[*] Crosswalk", RED_LINE_FLAG)
                elif key == 'h':
                    hill_hard_code()
                elif key == '\x03': # Ctrl+C
                    break
                else:
                    twist_msg.angular.z = 0
                
                mov_pub.publish(twist_msg)
                rospy.sleep(0.2)
                twist_msg.linear.x = 0
                twist_msg.angular.z = 0
                mov_pub.publish(twist_msg)


        except KeyboardInterrupt:
            print("Shutting down")

        finally:
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            mov_pub.publish(twist_msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    else: # competition
        license_pub.publish(START_TIMER)
        firstStrokeDrive()
        PID_DRIVE_FLAG = True
        rospy.spin()