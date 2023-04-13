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

        # define dictionaries
        self.characters = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'
        self.int_to_char = dict((i,c) for i,c in enumerate(self.characters))
        self.parkpos = '12345678'
        self.int_to_pos = dict((i,c) for i,c in enumerate(self.parkpos))

        # load the model
        


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

        # Save the image DELETE
        #cv2.imwrite('camImg.jpg', camImg)

        lic_plate, parkpos = self.seePlate(camImg)

        if (lic_plate and parkpos is not None):
            plate, position = self.decipherPlate(self, lic_plate, parkpos)
            print("plate = {} in P{}".format(plate, position))

        return None

    # Make the main running code here
    def seePlate(camImg):
        warped = self.getPlate(camImg)
        # define l_blue and u_blue
        l_blue = np.array([80, 140, 80])
        u_blue = np.array([150, 255, 235])
        if warped is not None:
            mask_p, blue_p = self.readPlate(warped, l_blue, u_blue) #define blues!
            #If successfully read license plate, then get parking. Otherwise exit
            plate_read = self.get_chars(mask_p, mask_p)
            if plate_read is not None:
                cropped_plate = get_chars(mask_p, mask_p)
                # Parking
                parking_plate = warped
                l_black = np.array([0, 0, 0])
                u_black = np.array([180, 255, 30])
                hsv_b = cv2.cvtColor(parking_plate, cv2.COLOR_BGR2HSV)
                mask_b = cv2.inRange(hsv_b, l_black, u_black)
                parking_pos = self.getParking(mask_b, mask_b)
            else 
                return None
        else
            return None

        return cropped_plate, parking_pos



    # Identify and get the plate 
    # Returns warped image (perspective transform)
    def getPlate(self, img):
        h = img.shape[0] #720
        w = img.shape[1] #1280

        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # Threshold the image to get only the blue color
        mask = cv2.inRange(hsv_img, lower_blue, upper_blue) #binary

        # Crop the images to remove the sky
        img = img[int(h/4):h-int(h/6), int(w/20):w-int(w/20)]
        mask = mask[int(h/4):h-int(h/6), int(w/20):w-int(w/20)]
        # Get the contours
        pic, coords = get_corners(mask, img)
        print(coords)

        if coords is None:
            return None
        else:
            # Perspective transform
            warped = transform(img, coords)

        return warped

    # Function that gets the contours (2 blue rectangles)
    # Params: masked image and original image
    # Returns an image with marked edges, coordinates
    def get_corners(self, mask, ori):
        img = ori.copy()
        # find contours
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # get the area of the entire image
        image_area = mask.shape[0] * mask.shape[1]

        #----
        # find the 3 largest contours
        largest_contours = []
        for c in sorted(contours, key=cv2.contourArea, reverse=True)[:3]:
            largest_contours.append(c)
            
        # exit if no 3 largest contours are found
        if len(largest_contours) < 3:
            print('Error: Could not find 3 largest contours')
            return None, None

        # find the 2 contours with the most similar height
        sorted_contours = sorted(largest_contours, key=lambda c: cv2.boundingRect(c)[3], reverse=True) #sort based on height
        height_diffs = [abs(cv2.boundingRect(sorted_contours[i])[3] - cv2.boundingRect(sorted_contours[i+1])[3]) for i in range(len(sorted_contours)-1)]
        most_similar_index = height_diffs.index(min(height_diffs)) if len(height_diffs) > 0 else None

        # exit if less than 2 largest contours are found
        if most_similar_index is None:
            print('Error: Could not find 2 largest contours with similar height')
            return None, None
        # take the 2 contours with the most similar height

        contours_to_use = sorted_contours[most_similar_index:most_similar_index+2]

        # check if there is at least one of the largest contours that is more than 1.2% of the entire image area
        largest_contours_area = [cv2.contourArea(c) for c in contours_to_use]
        if all(area < 0.012 * image_area for area in largest_contours_area):
            print('Error: None of the largest contours is more than 1.2% of the entire image area')
            return None, None
        # get the y-axis positions of the chosen contours
        y_positions = [cv2.boundingRect(c)[1] for c in contours_to_use]

        # get the y-axis positions and heights of the chosen contours
        y0, h0 = cv2.boundingRect(contours_to_use[0])[1:3]
        y1, h1 = cv2.boundingRect(contours_to_use[1])[1:3]

        # check if the lowest or highest y-axis of a contour is between the y-axis position of the other contour
        if not ((y0 <= y1 <= y0 + h0) or (y0 <= y1 + h1 <= y0 + h0) or
                (y1 <= y0 <= y1 + h1) or (y1 <= y0 + h0 <= y1 + h1)):
            print('Error: The y-axis positions of the chosen contours do not overlap')
            return None, None

        # exit if the 2 largest contours don't have similar height
        heights = [cv2.boundingRect(c)[3] for c in contours_to_use]
        if abs(heights[0] - heights[1]) > 40:
            print('Error: Contours do not have similar height')
            return None, None

        # get the x-axis positions and widths of the chosen contours
        x0, w0 = cv2.boundingRect(contours_to_use[0])[0:2]
        x1, w1 = cv2.boundingRect(contours_to_use[1])[0:2]

        # get the image width
        img_width = img.shape[1]

        # check if the x-axis distance of the two contours is more than half the image width
        if abs(x1 + w1 - x0) > img_width/2:
            print('Error: The x-axis distance of the chosen contours is too far apart')
            return None, None

        corners = []
        for rec in contours_to_use:
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

        print(corners)
        coords = get_left_right_rectangles(corners)

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

    def transform(self, img, src_coords):
        # Convert coordinates to np.float32 format
        src = np.float32(src_coords)
        dst = np.float32([[0, 0], [500,0], [500,500], [0, 500]])
        
        # Compute the perspective transform matrix and apply it to the image
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(img, M, (500, 500))

        return warped


    def readPlate(self, warped_img, l_blue, u_blue):
        plate = warped_img
        h = plate.shape[0]
        w = plate.shape[1]
        plate = plate[int(h/2):h, 5:(w-5)] # Cropped to remove blue residues on the edges
        kernel = np.ones((5,5), np.uint8)
        hsv_plate = cv2.cvtColor(plate, cv2.COLOR_BGR2HSV)

        mask_p = cv2.inRange(hsv_plate, l_blue, u_blue) #binary

        # optional (dilation to thicken the chars)
        #d ilation = cv2.dilate(mask_p,kernel,iterations = 1)

        # Apply the mask to the original image to extract the blue part
        blue_p = cv2.bitwise_and(plate, plate, mask = mask_p) #colour

        # Apply morphology operations to enhance the image quality
        mask_morph = cv2.morphologyEx(mask_p, cv2.MORPH_CLOSE, kernel) 
        blue_morph = cv2.morphologyEx(blue_p, cv2.MORPH_CLOSE, kernel) 

        return mask_morph, blue_morph

    def resize_image(self, image, new_width, new_height):
        height = image.shape[0]
        width = image.shape[1]

        if new_width is None and new_height is None:
            return image

        if new_width is None:
            aspect_ratio = new_height / float(height)
            new_width = int(width * aspect_ratio)
        elif new_height is None:
            aspect_ratio = new_width / float(width)
            new_height = int(height * aspect_ratio)

        resized_img = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
        resized_img = cv2.medianBlur(resized_img, 5)

        return resized_img
        
    # Params: masked image and original image (blue image)
    # Returns the 4 cropped images of each character
    def get_chars(self, mask, ori):
        img = ori.copy()
        # find contours
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # get the area of the entire image
        image_area = mask.shape[0] * mask.shape[1]

        # find the 4 largest contours
        largest_contours = []
        for c in sorted(contours, key=cv2.contourArea, reverse=True)[:4]:
            largest_contours.append(c)
            
        # exit if no 4 largest contours are found
        if len(largest_contours) < 4:
            print('Error: Could not find 4 characters')
            return None

        # exit if the 4 largest contours don't have similar height or width
        heights = [cv2.boundingRect(c)[3] for c in largest_contours]
        if abs(heights[0] - heights[1]) > 20:
            print('Error: Chars do not have similar height')
            return None
        widths = [cv2.boundingRect(c)[2] for c in largest_contours]
        if abs(widths[0] - widths[1]) > 20:
            print('Error: Chars do not have similar width')
            return None

        # exit if any contour has an area less than 28% of any of the other contours
        for i, c1 in enumerate(largest_contours):
            for c2 in largest_contours[i+1:]:
                area_ratio = cv2.contourArea(c1) / cv2.contourArea(c2)
                if area_ratio < 0.28 or area_ratio > 28:
                    print('Error: Contour area ratio is less than 28%')
                    return None, None
                
        # check if the area of one of the largest contours is less than 0.05% of the entire image area
        for c in largest_contours:
            contour_area = cv2.contourArea(c)
            if contour_area < 0.0005 * image_area:
                print('Error: Area of one of letter is less than 0.05% of the entire image area')
                return None

        corners = []
        x_coords = []
        for rec in largest_contours:
            # get the bounding rectangle coordinates
            x, y, w, h = cv2.boundingRect(rec)
            tl = (x, y)
            tr = (x+w, y)
            br = (x+w, y+h)
            bl = (x, y+h)
            corners.append((tl, tr, br, bl))

            x_coords.append((tl[0], tr[0]))


        sorted_x_coords = sorted(x_coords, key=lambda x: x[0])
        print(sorted_x_coords)

        # Get max and min diff of char length
        min_diff = 500
        max_diff = 0
        for tup in sorted_x_coords:
            diff = abs(tup[0] - tup[1])
            if diff < min_diff:
                min_diff = diff
            if diff > max_diff:
                max_diff = diff
        #print(min_diff, max_diff)
        
        # Get the left x coordinate
        left_x = []
        for x in sorted_x_coords:
            left_x.append(x[0])

        lowest_y = sorted(corners, key=lambda x: x[0][1])[0][0][1]

        max_height = 0
        for corner in corners:
            diff = corner[2][1] - lowest_y
            if diff > max_height:
                max_height = diff

        cropped_imgs = []
        for i, x in enumerate(left_x):
            cropped_img = ori[lowest_y:lowest_y+max_height, x:x+max_diff]
            cropped_imgs.append(cropped_img)

        if cropped_imgs is not None:
            for i in range(len(cropped_imgs)):
            cv2_imshow(cropped_imgs[i])
        else:
            print('Error: Could not find characters')
            return None

        return cropped_imgs

    # Params: masked image and original image
    # Returns the parking character
    def getParking(mask, ori):
        img = ori.copy()
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contours = []
        for c in sorted(contours, key=cv2.contourArea, reverse=True)[:2]:
            largest_contours.append(c)
            
        if len(largest_contours) < 2:
            print('Error: Could not find the parking position')
            return None, None

        heights = [cv2.boundingRect(c)[3] for c in largest_contours]
        if abs(heights[0] - heights[1]) > 20:
            print('Error: Parking position do not have similar height; are not characters')
            return None, None

        corners = []
        x_coords = []
        for rec in largest_contours:
            x, y, w, h = cv2.boundingRect(rec)
            tl = (x, y)
            tr = (x+w, y)
            br = (x+w, y+h)
            bl = (x, y+h)
            corners.append((tl, tr, br, bl))

            x_coords.append((tl[0], tr[0]))

        sorted_x_coords = sorted(x_coords, key=lambda x: x[0])
        print(sorted_x_coords)

        # Get max and min diff of char length
        min_diff = 500
        max_diff = 0
        for tup in sorted_x_coords:
            diff = abs(tup[0] - tup[1])
            if diff < min_diff:
                min_diff = diff
            if diff > max_diff:
                max_diff = diff
        
        # Get the left x coordinate
        left_x = []
        for x in sorted_x_coords:
            left_x.append(x[0])

        lowest_y = sorted(corners, key=lambda x: x[0][1])[0][0][1]

        # Get the max height by finding the maximum y coordinate difference starting from the lowest y coordinate
        max_height = 0
        for corner in corners:
            diff = corner[2][1] - lowest_y
            if diff > max_height:
                max_height = diff

        cropped_imgs = []
        for i, x in enumerate(left_x):
            cropped_img = ori[lowest_y:lowest_y+max_height, x:x+max_diff]
            cropped_imgs.append(cropped_img)

        return cropped_imgs[1] 


    # Test CNN
    # Params: lic_img (license plate image from get_chars) and p_img (pos img from getParking)
    # Returns the plate and position (string)
    def decipherPlate(lic_img, p_img):
    plate = ""
    pos = ""
    # Reading the plate
    for i in range(len(lic_img)):
        img = lic_img[i]
        char =  tf.expand_dims(img, axis=0)
        with self.graph.as_default():
        try:
            set_session(self.sess)
            y_pred = self.conv_model.predict(char)[0]
            plate = plate + self.int_to_char[np.argmax(y_pred)]
        except Exception as e:
            #print("No plate found")

    # Reading the position
    pos_num = tf.expand_dims(p_img, axis = 0)
    with self.graph.as_default():
        try:
        set_session(self.sess)
        pos_pred = self.conv_model.predict(pos_num)[0]
        pos = self.int_to_pos[np.argmax(pos_pred)]
            
    return pos, plate


    
# # Create object; it's wrong -> shouldn't be in a class
# bridge = Cv.Bridge()
# image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, callback) #subscribe calls the callback func like an interrupt
# #license_pub = rospy.Publisher("license_plate", String, queue_size=10)

# cv.destroyAllWindows()


# Create the object
licensePlates = LicensePlate()
rospy.spin() #has infinite loop
