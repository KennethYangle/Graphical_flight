#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from geometry_msgs.msg import Point, PoseStamped 


import io
import os
import time
import json5



import numpy as np
from scipy.optimize import linear_sum_assignment




file_pwd = os.path.dirname(os.path.abspath(__file__))
time_prefix = time.strftime("%Y-%m-%d_%H-%M-%S")

import sys
__BASE__ = os.path.dirname(os.path.dirname(file_pwd))
sys.path.append(__BASE__+"/Graphical")
from main_ly import useGTA


class Allocation:
    def __init__(self):
        mav_num = rospy.get_param('mav_num', 10)
        self.mav_posL = np.zeros((mav_num,2))
        self.Pcur = np.zeros((mav_num,3))
        self.ros_subL = [
            rospy.Subscriber("/vicon/AA_rfly_{:01d}/pose".format(i+1), PoseStamped, self.mav_pos_callback,(i,))
            for i in range(mav_num)    
        ]
        
    
    def mav_pos_callback(self,msg,i):
        self.mav_posL[i,:] = msg.pose.position.x, msg.pose.position.y
        self.Pcur[i,:] = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
    
    def allocate(self,circle_posL):
        print("mav_posL:")
        print(self.mav_posL)
        cost_mat = [
            [np.linalg.norm(mav_pos-circle_pos) for circle_pos in circle_posL]
            for mav_pos in self.mav_posL
        ]
        _,task_result = linear_sum_assignment(cost_mat)

        return task_result

    def allocate_yk(self, circle_posL, mav_id):
        print("Pcur:", self.Pcur)
        p_search = np.array(circle_posL)
        print("p_search:", p_search)
        N = max(np.size(self.Pcur, 0), np.size(p_search, 0))
        ViewR = np.array([4000 for i in range(N)])
        p_next = useGTA(self.Pcur, ViewR, p_search)
        print("p_next:", p_next)

        return p_next[mav_id-1]
