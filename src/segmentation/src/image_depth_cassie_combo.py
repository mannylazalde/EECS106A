#!/usr/bin/env python


# BIG TAKEAWAY: sourcing camerawork moves PYTHONPATH (echo $PYTHONPATH) to ros kinetic directory
#               instead of the proper location

import os
import numpy as np
import matplotlib.pyplot as plt
import time
import rospy
from sensor_msgs.msg import Image as msg_image
from geometry_msgs.msg import Vector3
import message_filters

from cv_bridge import CvBridge, CvBridgeError

import sys
#change path to insert correct version of cv2
#sys.path.insert(0,'/home/eecs106a/.local/lib/python2.7/site-packages')

import cv2



 # Create a CvBridge to convert ROS messages to OpenCV images
#bridge = CvBridge()


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

            if radius > 15:
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


def parabolic(radius):
    a0 = 2.59936585e+01
    a1 = -3.72791626e-01
    a2 = 1.32533945e-03
    return a2*radius*radius + a1*radius + a0

def get_height(radius):
    return parabolic(radius)


# Converts a ROS Image message to a NumPy array to be displayed by OpenCV
def ros_to_cv2_img(ros_img_msg):
	return np.array(bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))


### ROS subscriber nonsense 
def callback(message):

	#converts from ROS format to cv2 format
    image = ros_to_cv2_img(message)

    
    #cv2.imwrite("7.jpeg", image)
    print(cv2.__version__)

    #EDIT THIS TO LOAD IMAGE IN TRACK
    X, Y, img1, radius = ball_tracker.track(image)

    Z = get_height(radius)

    camera_pub.publish(Vector3(X,Y,Z))

    # print("image save")
    cv2.imshow("Image window", img1)
    cv2.waitKey(1)


    #X,Y,Z should be changed based on image processing

    


#Define the method which contains the node's main functionality
def listener():

    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    rospy.init_node('Image_Processor', anonymous=True)

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.


    #/camera/color/image_raw - Should be something along this line

    ############################### Might have to change on Cassie##################
    rospy.Subscriber("/device_0/sensor_1/Color_0/image/data", msg_image, callback)

    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()

def main():

    z0 = 103.1875 #[mm]
    r0 = 153.6826171875

    r1 = (60.24595642089844 + 61.16786193847656)/2
    z1 = z0 + 203.2 #[mm]

    r2 = 38.5520133972168
    z2 = z0 + 381 #[mm]

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

    # Configure the depth and colour streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Intrinsic matrix
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()

    # Get the depth sensor's depth scale
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Align the depth stream to the RGB stream
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Create a camera node
    rospy.init_node('ball_tracker', anonymous=True)
    camera_pub = rospy.Publisher('ball_position', Vector3, queue_size=10)

    # Set loop rate
    loop_rate = 600
    dt = 1/loop_rate
    rate = rospy.Rate(loop_rate)

    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue

        color_frame = frames.get_color_frame()
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_intrinsics = rs.video_stream_profile(
        aligned_frames.profile).get_intrinsics()


        X, Y,img1= ball_tracker.track(color_image)
        color_image = ball_tracker.draw_arrows(color_image)
        Depth = rs.depth_frame.get_distance(aligned_depth_frame, X, Y)
        X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [X, Y], Depth)
        camera_pub.publish(Vector3(X,Y,Z))

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((color_image, depth_colormap))
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(images, 'Y: %.4f, X: %.4f Depth: %.4f'%(Y, X, Z), (10,450), font, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)

        key = cv2.waitKey(30)
        if key == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
        rate.sleep()
    pipeline.stop()


if __name__ == '__main__':
    main()

   

