#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import time
import cv2
import numpy as np

import rospy
import threading


from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError



from rgbd_ring_proc import single_centre_ring_dect








img_w,img_h = 640,480
depth_w,depth_h = 640,480
# 深度图算距离，RGB图算坐标
colorK = np.mat(
    [
        [610.96,0,312.11],
        [0,610.08,240.94],
        [0,0,1],
    ],
    dtype=float
)



class rgbd_img_proc:
    def __init__(self):
        
        self.img_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.res_bridge = CvBridge()
        
        self.circle_xyr = [-1]*3
        self.circle_FRD = [-1.0]*3
        self.circle_FRD_px = [-1.0]*3

        self.color_done = False
        self.depth_done = False

        self.color_img = None
        self.depth_img = None
        self.depth_img_uint8 = None

        self.color_lock = threading.Lock()
        self.depth_lock = threading.Lock()

        
        self.img_cnt = 0
        self.circle_info_time = time.time()

        
        self.img_sub = rospy.Subscriber("/d435i/color/image_raw", ImageMsg, self.img_cb)
        self.depth_sub = rospy.Subscriber("/d435i/aligned_depth_to_color/image_raw", ImageMsg, self.depth_cb)

        self.img_res_pub = rospy.Publisher("/bs_debuger/image_res", ImageMsg, queue_size=1)
    
    def reset(self):
        self.circle_xyr = [-1]*3
        self.circle_FRD = [-1.0]*3
        self.circle_FRD_px = [-1.0]*3


    # def init_done(self):
    #     return self.color_done and self.depth_done

    def img_cb(self,msg):
        if not self.color_done:
            self.color_done = True
        self.color_lock.acquire()
        cv_image = self.img_bridge.imgmsg_to_cv2(msg, msg.encoding)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.color_img = cv2.rotate(cv_image, cv2.ROTATE_180)
        self.color_lock.release()
    
    def depth_cb(self,msg):
        if not self.depth_done:
            self.depth_done = True
        self.depth_lock.acquire()
        cv_image = self.img_bridge.imgmsg_to_cv2(msg, msg.encoding)
        self.depth_img = cv2.rotate(cv_image, cv2.ROTATE_180)
        self.depth_lock.release()
    
   

    
    def update_ring_info(self):
        result = False
        if not self.color_done or not self.depth_done:
            return result

        self.depth_lock.acquire()
        self.color_lock.acquire()

        img_result = single_centre_ring_dect(self.color_img,self.depth_img)
        cx,cy,r,real_dis = img_result
        if -1 in [cx,cy]:
            self.circle_xyr = [-1.0]*3
            self.circle_FRD = [-1.0]*3
        else:
            p_rd = np.array([[cx,cy] + [1]])
            p_out1 = colorK.I*p_rd.T
            R,D,F = (p_out1.T*real_dis).tolist()[0]

            self.circle_xyr = [cx,cy,r]
            self.circle_FRD = [F,R,D]

        img_msg = self.res_bridge.cv2_to_imgmsg(self.color_img, "bgr8")
        self.img_res_pub.publish(img_msg)

        self.color_lock.release()
        self.depth_lock.release()
        self.color_done = False
        self.depth_done = False
        self.circle_info_time = time.time()
        return result



