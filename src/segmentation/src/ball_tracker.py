#!/usr/bin/env python

import os
import numpy as np
import cv2

# Define file path & image directory
this_file = os.path.dirname(os.path.abspath(__file__))
IMG_DIR = '/'.join(this_file.split('/')[:-2]) + '/img'

class Tracker:
    def __init__(self, height, width, color_lower, color_upper):
        self.color_lower = color_lower
        self.color_upper = color_upper
        self.midx = int(width / 2)
        self.midy = int(height / 2)
        self.xoffset = 0
        self.yoffset = 0

    def draw_arrows(self, frame):
        return frame

    def track(self, frame):
        # Apply Gaussian Blur to filter image and blur out noise
        blurred = cv2.GaussianBlur(frame, (5, 5), 0) #pass in image, kernel size, and std dev
        # Convert blurred image from BGR to HSV (hue, saturation, value) color mode
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # Threshold the HSV image using the low and high HSV threshold values
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        # Erode away boundaries of regions of foreground pixels (usually white)
        mask = cv2.erode(mask, None, iterations=2)
        # Expand the boundaries of regions of foreground pixels (usually white)
        mask = cv2.dilate(mask, None, iterations=2)
        # Show the image (now named mask) after applying filters
        #cv2.imshow('mask', mask)
        # Find contours in image (image, contour retrieval mode, contour approximation method)
        # Contours stored as vectors of points, hierarchy is unused
        contours, hierarchy= cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Initialize the centroid of the object
        center = None

        if len(contours) > 0:
            # Find the contour with the greatest contour area
            c = max(contours, key=cv2.contourArea)
            # Find a circle of the minimum area enclosing the contour points
            # Outputs (x,y) center coordinates and radius of circle
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            # Get dictionary of all moment values calculated
            M = cv2.moments(c) 
            # Calculate the centroid of the object
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 1:
                # Draw a circle outlining the object (image, center, radius, color, thickness)
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # Draw a circle tracking the centroid of the object
                # Negative thickness creates a filled circle
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                # Set the offset values to the current centroid
                self.xoffset = int(center[0])
                self.yoffset = int(center[1])
            else:
                self.xoffset = -1
                self.yoffset = -1
        else:
            self.xoffset = -1
            self.yoffset = -1
        return self.xoffset, self.yoffset, frame

def main():
    #EDIT THIS TO READ IMAGE
    #color_image = cv2.imread(IMG_DIR + '/08in_dark_Color.png')
    color_image1 = cv2.imread('00in_light_Color.png')

    # Initialize lower and upper HSV threshold values for ping pong ball
    H_scale = 255/360.0 #H values span from 0-360
    S_V_scale = 255/100.0 #S and V values span from 0-100
    
    #CHANGE THESE HSV VALUES
    H_low = 0
    S_low = 16
    V_low = 44
    H_high = 20
    S_high = 100
    V_high = 100

    #Use HSV Color Picker values and scale to 0-255 range
    ball_lower = (int(H_low*H_scale), int(S_low*S_V_scale), int(V_low*S_V_scale))
    ball_upper = (int(H_high*H_scale), int(S_high*S_V_scale), int(V_high*S_V_scale))

    # Create a ball tracker
    ball_tracker = Tracker(1280, 720, ball_lower, ball_upper)
    #EDIT THIS TO LOAD IMAGE IN TRACK
    X, Y, img1 = ball_tracker.track(color_image1)

    #EDIT THIS TO SHOW IMAGE
    cv2.imshow("track1", img1)
    cv2.waitKey(0)

if __name__ == '__main__':
    main()
