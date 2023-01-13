#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

import io
import os
import time
import numpy as np

import json5

from rgbd_proc import rgbd_img_proc
from visual_servo_bs import visual_servo_bs


from utils import constrain_rad

file_pwd = os.path.dirname(os.path.abspath(__file__))
time_prefix = time.strftime("%Y-%m-%d_%H-%M-%S")


task_json = "{}/config.json".format(file_pwd)
with io.open(task_json,"r",encoding="utf-8") as fp:
    dataD = json5.load(fp)

circle_posL = dataD["circle_posL"]
vs_dict = dataD["vs_dict"]



debug_nameL = [
    "task_state",
    "yolo_px"   ,
    "yolo_frd"  ,
    "cmdv_flu"  ,
    "allocation"  ,
]

class Debuger:
    def __init__(self):
        self.pub_dict = {}
        for i in debug_nameL:
            self.pub_dict[i] = rospy.Publisher('/bs_debuger/{}'.format(i), String, queue_size=1)

    def update(self,debug_info_dict):
        for k,v in debug_info_dict.items():
            self.pub_dict[k].publish(v)




TASK_ACCESS     = 0
TASK_VISUAL     = 1
TASK_THROUGH    = 2

TASK_ALLOCATION = 11
TASK_SWARM_POS  = 12


class TaskCircleThree:
    def __init__(self,px4_control,target_allocater):
        self.px4_control = px4_control
        self.target_allocater = target_allocater

    
    def run(self):
        px4_control = self.px4_control
        target_allocater = self.target_allocater


        img_proc = rgbd_img_proc()
        visual_servo = visual_servo_bs()

        debuger = Debuger()


        task_state = TASK_ALLOCATION
        mav_id = rospy.get_param('~mav_id', 1)

        print("TaskCircleThree mav_id:{}".format(mav_id))
        

        v_cmd = [0]*3
        tgt_yaw = 0
        visual_servo.reset()
        visual_servo.__dict__.update(vs_dict)


        tgt_circle_pos_idx = -1
        tgt_circle_pos = [0]*3

        while True:

            yaw = px4_control.mav_yaw_odom
            R_be = px4_control.R_be
            pos_swarm = px4_control.pos_swarm

            img_proc.update_ring_info()


            if task_state == TASK_ALLOCATION:
                # tgt_taskL = target_allocater.allocate(circle_posL)
                # tgt_circle_pos_idx = tgt_taskL[mav_id-1]
                # tgt_circle_pos = circle_posL[tgt_circle_pos_idx]
                # print("tgt_taskL: {}".format(tgt_taskL))
                # print("tgt_circle_pos_idx: {}".format(tgt_circle_pos_idx))
                tgt_circle_pos = target_allocater.allocate_yk(circle_posL, mav_id)
                print("tgt_circle_pos: {}".format(tgt_circle_pos))
                task_state = TASK_SWARM_POS
            elif task_state == TASK_SWARM_POS:
                px4_control.moveBySwarmPosEN(x=target_allocater.Pcur[mav_id-1][0], y=tgt_circle_pos[1])
                task_state = TASK_ACCESS
            elif task_state == TASK_ACCESS:
                # px4_control.moveByVelocityYawrateBodyFrame(vx=0.5)
                px4_control.moveByVelocityYawrateENU(vx=0.5)
                if visual_servo.switch_to(img_proc.circle_xyr[2]):
                    task_state = TASK_VISUAL
                    print("visual servo start")
            elif task_state == TASK_VISUAL:
                visual_servo.update_kinematics(R_be,yaw)
                v_cmd, yaw_rate_cmd = visual_servo.visual_servo_real_update(img_proc.circle_xyr,img_proc.circle_FRD,tgt_yaw)
                
                px4_control.moveByVelocityYawrateBodyFrame(v_cmd[0],v_cmd[1],v_cmd[2],0)
                # px4_control.moveByVelocityYawrateENU(v_cmd[0],v_cmd[1],v_cmd[2],yaw_rate_cmd)
                if visual_servo.next_task_flag:
                    break
            debug_info_dict = {
                "task_state": (
                    "s:{} vs:{}\n".format(task_state,visual_servo.vs_state) +
                    "yolo_xyr:{:.2f} {:.2f} {:.2f}\n".format(img_proc.circle_xyr[0],img_proc.circle_xyr[1],img_proc.circle_xyr[2],) +
                    "yolo_frd:{:.2f} {:.2f} {:.2f}\n".format(img_proc.circle_FRD[0],img_proc.circle_FRD[1],img_proc.circle_FRD[2],) +
                    "pos_swarm:{:.2f} {:.2f} {:.2f}\n".format(pos_swarm[0],pos_swarm[1],pos_swarm[2])
                ),
                "yolo_px"   : "yolo_px:{:.2f} {:.2f} {:.2f}".format(img_proc.circle_xyr[0],img_proc.circle_xyr[1],img_proc.circle_xyr[2]),
                "yolo_frd"  : "yolo_frd:{:.2f} {:.2f} {:.2f}".format(img_proc.circle_FRD[0],img_proc.circle_FRD[1],img_proc.circle_FRD[2]),
                "cmdv_flu"  : "v_flu:{:.2f} {:.2f} {:.2f}".format(v_cmd[0],v_cmd[1],v_cmd[2]),
                "allocation"  : "tgt_circle_pos_idx:{} tgt_circle_pos:{}".format(tgt_circle_pos_idx,tgt_circle_pos),
            }
            debuger.update(debug_info_dict)
            time.sleep(0.01)




        
