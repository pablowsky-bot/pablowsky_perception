#!/usr/bin/python3

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from dynamic_reconfigure.server import Server
from pablowsky_perception.cfg import hsvBoundsConfig

class HSVColorFilter:
    def __init__(self):
        rospy.loginfo('hsv color filter node started')
        self.img_flag = False
        rospy.Subscriber('webcam_img', Image, self.imgCallback)
        self.pub = rospy.Publisher('color_filtered_img', Image, queue_size=10)
        self.br = CvBridge()
        self.min_h = 0
        self.max_h = 255
        self.min_s = 0
        self.max_s = 255
        self.min_v = 0
        self.max_v = 255

    def imgCallback(self, msg):
        self.img_flag = True
        # Used to convert between ROS and OpenCV images
        self.img = self.br.imgmsg_to_cv2(msg)

    def reconfigureCB(self, config, level):
        rospy.loginfo('hsv reconfigure request received')
        self.min_h = config['min_h']
        self.max_h = config['max_h']
        self.min_s = config['min_s']
        self.max_s = config['max_s']
        self.min_v = config['min_v']
        self.max_v = config['max_v']
        return config

    def start(self):
        srv = Server(hsvBoundsConfig, self.reconfigureCB)
        rate = rospy.Rate(10) #10Hz - ten times per second
        while not rospy.is_shutdown():
            if self.img_flag:
                self.img_flag = False                
                # changing order: from rgb to bgr (bgr is what opencv uses)
                bgr = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
                # converting bgr to hsv
                hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

                # using threshold for color orange
                mask = cv2.inRange(hsv,(self.min_h, self.min_s, self.min_v), (self.max_h, self.max_s, self.max_v))

                out = self.br.cv2_to_imgmsg(mask)
                # out.header.frame_id = "map"

                self.pub.publish(out)
            # Sleep to maintain the desired node rate
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hsv_color_filter_node')
    hsv_color_filter_node = HSVColorFilter()
    hsv_color_filter_node.start()
