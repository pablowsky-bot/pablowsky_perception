#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def pub_image():
    pub = rospy.Publisher('webcam_img', Image, queue_size=10)
    
    rate = rospy.Rate(10) #10Hz - ten times per second
    
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    cap = cv2.VideoCapture('/dev/video4') # NOTE: change for your webcam! or use 0 for default

    # Used to convert between ROS and OpenCV images
    br = CvBridge()
    
    while not rospy.is_shutdown():
        # Capture frame by frame
        # method returns true/false and video frame
        ret, frame = cap.read()
        if ret:
            out = br.cv2_to_imgmsg(frame)
            out.header.frame_id = "map"
            pub.publish(out)
        # Sleep to maintain the desired node rate
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('webcam_img_pub')
    pub_image()
