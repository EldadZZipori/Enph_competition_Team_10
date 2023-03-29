#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dense
from keras.optimizers import Adam
from sklearn.model_selection import train_test_split

IMAGE_SIZE = (480, 640)
TRAINING_DATA_DIR = 'training_data'
BATCH_SIZE = 32
NUM_EPOCHS = 10

class ImageCapture:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.velocity_sub = rospy.Subscriber("/cmd_vel", Twist, self.velocity_callback)
        self.images = []
        self.velocities = []
        self.is_capturing_data = False
        
    def image_callback(self, data):
        if self.is_capturing_data:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.resize(cv_image, IMAGE_SIZE)
            self.images.append(cv_image)
            
    def velocity_callback(self, data):
        if self.is_capturing_data:
            self.velocities.append([data.linear.x, data.angular.z])
            
    def start_capturing_data(self):
        self.images = []
        self.velocities = []
        self.is_capturing_data = True
        
    def stop_capturing_data(self):
        self.is_capturing_data = False
        
    def save_data_to_disk(self, filename):
        if len(self.images) == 0:
            return
        X = np.array(self.images)
        y = np.array(self.velocities)
        np.savez(filename, X=X, y=y)
        self.images = []
        self.velocities = []
        
def build_model(input_shape):
    model = Sequential()
    model.add(Conv2D(32, (3, 3), activation='relu', input_shape=input_shape))
    model.add(MaxPooling2D((2, 2)))
    model.add(Conv2D(64, (3, 3), activation='relu'))
    model.add(MaxPooling2D((2, 2)))
    model.add(Conv2D(128, (3, 3), activation='relu'))
    model.add(MaxPooling2D((2, 2)))
    model.add(Flatten())