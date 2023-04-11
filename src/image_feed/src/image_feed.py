#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np 
from sensor_msgs.msg import Image
from std_mesgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf
from sklearn.model_selection import train_test_split

from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers
from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend
from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model


class LicensePlate():

    #initiate the object
    def __init__(self):

        tf.keras.backend.clear_session()


        self.bridge = CvBridge()
        self.imageSubscriber = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.findandread)
        self.ReadPublisher = rospy.Publisher('/license_plate', String, queue_size = 10)
        self.ReadRate = rospy.Rate(10)
        self.prevError = 0

        #CNN
        

    # Read image
    def callback(self, data):
        try: 
            camImg = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Save the image
        cv2.imwrite('camImg.jpg', camImg)

        #license_plate = self.getPlate(camImg)
        #if(license_plate is not None):


        # Get the dimensions
        h, w, c = camImg.shape

        return None

    # Identify and get the plate 
    # Main function to get the license plate
    # Run after running all other cells
    def getPlate(self, img):
        # Convert the image to HSV color space
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #cv2_imshow(hsv_img)
        # Threshold the image to get only the blue color
        mask = cv2.inRange(hsv_img, lower_blue, upper_blue) #binary
        #cv2_imshow(mask)

        m_h = mask.shape[0] # h= 1479
        m_w = mask.shape[1] # w = 2256

        # Get the contours
        pic, coords = get_corners(mask, img)
        print(coords)
        # Perspective transform
        warped = transform(img, coords)

        #cv2_imshow(warped)
        return None

    # Function that gets the contours (2 blue rectangles)
    # Params: masked image and original image
    # Returns an image with marked edges
    def get_corners(self, mask, ori):
        img = ori.copy()
        # find contours
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)

        # find the 2 largest contours
        largest_contours = []
        for c in sorted(contours, key=cv2.contourArea, reverse=True)[:2]:
            largest_contours.append(c)

        corners = []
        for rec in largest_contours:
            # get the bounding rectangle coordinates
            x, y, w, h = cv2.boundingRect(rec)
            # draw the bounding rectangle
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            # Get coordinates
            tl = (x, y)
            tr = (x+w, y)
            br = (x+w, y+h)
            bl = (x, y+h)
            # Append coordinates to list
            corners.append((tl, tr, br, bl))

            print('Top-left corner: ({}, {})'.format(x, y))
            print('Bottom-right corner: ({}, {})'.format(x+w, y+h))
            print('Top-right corner: ({}, {})'.format(x+w, y))
            print('Bottom-left corner: ({}, {})'.format(x, y+h))
            #draw circles
            cv2.circle(img, tl, 20, (0, 0, 255), 2) #red top left
            cv2.circle(img, br, 20, (0, 255, 0), 2) #green bottom right
            cv2.circle(img, tr, 20, (255, 0, 0), 2) # blue top right
            cv2.circle(img, bl, 20, (100, 100, 100), 2) # bottom left

        print(corners)
        coords = get_left_right_rectangles(corners)

        # show img
        cv2_imshow(img)
        return img, coords

    def get_left_right_rectangles(self, corners):
        # Check if the left rectangle has the smaller x-coordinate
        rec1 = corners[0]
        rec2 = corners[1]
        print(rec1[0][0], 'and',rec2[0][0])
        # Check which is left or right rectangle through comparing the x coord of tl
        if rec1[0][0] < rec2[0][0]:
        left_rec = rec1
        right_rec = rec2
        else:
        left_rec = rec2
        right_rec = rec1

        left_tr = left_rec[1] #tl
        left_br = left_rec[2] #bl
        right_tl = right_rec[0] #tr
        right_bl = right_rec[3] #br
        coords = (left_tr, right_tl, right_bl, left_br)

        return coords

        # Function for perspective transform
        # The src_coords are from 'get_left_right_rectangles()'
        # we define the dst_coords 
        def transform(self, img, src_coords):
            # Convert coordinates to np.float32 format
            src = np.float32(src_coords)
            dst = np.float32([[0, 0], [500,0], [500,500], [0, 500]])
            
            # Compute the perspective transform matrix and apply it to the image
            M = cv2.getPerspectiveTransform(src, dst)
            warped = cv2.warpPerspective(img, M, (500, 500))
            # Display the original and warped images
            # cv2_imshow(img)
            cv2_imshow(warped)
            return warped

    
    # Train the CNN

    # One-hot encoding
    def one_hot_encode(label):
        integer_labels = {'0': 26, '1': 27, '2': 28, '3': 29, '4': 30, '5': 31, '6': 32, '7': 33, '8': 34, '9': 35,
                        'A': 0, 'B': 1, 'C': 2, 'D': 3, 'E': 4, 'F': 5, 'G': 6, 'H': 7, 'I': 8, 'J': 9, 'K': 10,
                        'L': 11, 'M': 12, 'N': 13, 'O': 14, 'P': 15, 'Q': 16, 'R': 17, 'S': 18, 'T': 19, 'U': 20,
                        'V': 21, 'W': 22, 'X': 23, 'Y': 24, 'Z': 25}
                        
        int_label = integer_labels[label]
        if int_label is None:
        raise ValueError(f"Invalid label: {label}")
        one_hot_label = np.zeros(36)
        one_hot_label[int_label] = 1

        return one_hot_label

    
# # Create object; it's wrong -> shouldn't be in a class
# bridge = Cv.Bridge()
# image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, callback) #subscribe calls the callback func like an interrupt
# #license_pub = rospy.Publisher("license_plate", String, queue_size=10)

# cv.destroyAllWindows()


# Create the object
licensePlates = LicensePlate()
rospy.spin() #has infinite loop
