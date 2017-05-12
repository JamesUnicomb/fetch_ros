#!/usr/bin/env python
import sys, time
import rospy
import rospkg

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

rospy.init_node('find_card_reader', anonymous=True)

class FindCardReader:
    def __init__(self):
        self.rgb_sub = rospy.Subscriber('head_camera/rgb/image_rect_color', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('head_camera/depth_registered/image_raw', Image, self.depth_callback)
        self.bridge = CvBridge()
        path_to_pkg = rospkg.RosPack().get_path('find_card_reader')
        template_rgb_img = cv2.imread(path_to_pkg + '/include/card_reader.JPG')
        self.depth_template = np.load(path_to_pkg + '/include/depth_template.npy')
        self.rgb_template = cv2.cvtColor(template_rgb_img, cv2.COLOR_BGR2GRAY)
        
    def rgb_callback(self, data):
        img_rgb = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.rgb_seq = data.header.seq
        self.rgb_array = img_rgb

    def depth_callback(self, data):
        img_depth = self.bridge.imgmsg_to_cv2(data)
        self.depth_array = img_depth

def main():
    seq = 0
    threshold = 0.1
    last_publish = 0

    rate_recieve = rospy.Rate(15)
    rate_wait = rospy.Rate(1)

    card_reader = FindCardReader()

    rgb_template = card_reader.rgb_template
    depth_template = card_reader.depth_template
    w, h = rgb_template.shape[::-1]

    while not rospy.is_shutdown():
        seq += 1
        try:
            img_rgb = card_reader.rgb_array
            if last_publish != card_reader.rgb_seq:
                last_publish = card_reader.rgb_seq
                img_depth = card_reader.depth_array
                img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
                res = cv2.matchTemplate(img_gray, rgb_template, cv2.TM_CCOEFF_NORMED)
                loc = np.where(res >= threshold)
                for pt in zip(*loc[::-1]):
                    depth_cut = np.array(img_depth[pt[0]:pt[0] + w, pt[1]:pt[1] + h]).flatten()
                    cv2.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0,255,0), 2)
                np.save('depth_image_template', img_depth)
                cv2.imshow('rgb_image', img_rgb)
                cv2.waitKey(500)
            cv2.destroyAllWindows()
            rate_recieve.sleep()
        except AttributeError:
            rate_wait.sleep()

if __name__=='__main__':
    main()  
