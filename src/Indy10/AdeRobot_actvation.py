#!/usr/bin/env python3
# -*- coding:utf-8 -*-

# Process of Robot
import rospy
from std_msgs.msg import String  # String 메시지를 처리하기 위한 헤더
from move_group_python_interface2 import MoveGroupPythonInterface
from math import tau


import time
from indy_driver.msg import warning, drink_info

# Process of Image
import cv2
import os

def drink_info_callback(msg):
    global drink_info_received
    drink_info_received = msg.name
    rospy.loginfo(f"Received drink_info: {msg.name}")

def warning_callback(msg):
    global warning_received
    warning_received = msg.name
    rospy.loginfo(f"Received warning: {msg.name}")

# Dont Work

def display_image(image_path):
    if os.path.exists(image_path):  # 경로가 실제 파일인지 확인
        image = cv2.imread(image_path)
        cv2.imshow("Warning Image", image)
        cv2.waitKey(1)  # 창이 유지되도록 설정


def main():
    try:

        # 구독자 초기화
        rospy.Subscriber('drink_info', drink_info, drink_info_callback)
        rospy.Subscriber('warning', warning, warning_callback)


        global warning_received, drink_info_received
        warning_received = ""
        drink_info_received = ""

        indy10 = MoveGroupPythonInterface(real=True, gripper="rg2")


        rate = rospy.Rate(10)  # 10 Hz 루프
        while not rospy.is_shutdown():
            if warning_received and drink_info_received:
                rospy.loginfo(f"[INFO] warning_received: {warning_received}, drink_info_received: {drink_info_received}")

                if warning_received == "0":
                    ## 공통부 1
                    display_image("/home/sionseo/catkin_ws/src/indy_driver/src/Normal.png")
                    input("============ Moving initial pose using a joint state goal ...")

                    indy10.move_to_standby()

                    rel_xyz = [-0.16, -0.20, 0.16]
                    rel_rpy = [0.0, 0.0, 0.0]
                    indy10.go_to_pose_rel(rel_xyz, rel_rpy)
                    
                    indy10.gripper.grip_on()

                    init_pose_joints = [(77.00/360)*tau, (13.72/360)*tau, (100.16/360)*tau, (-94.92/360)*tau, (74.55/360)*tau, (115.63/360)*tau]          # tau = 2 * pi
                    indy10.go_to_joint_abs(init_pose_joints)
                    
                    # 컵 위치로 이동
                    init_pose_joints = [(45.04/360)*tau, (36.04/360)*tau, (71.38/360)*tau, (-103.53/360)*tau, (46.83/360)*tau, (113.89/360)*tau]          # tau = 2 * pi
                    indy10.go_to_joint_abs(init_pose_joints)

                    indy10.gripper.grip_width(110)

                    
                    rel_xyz = [0.0, -0.35, 0.25]
                    rel_rpy = [0.0, 0.0, 0.0]
                    indy10.go_to_pose_rel(rel_xyz, rel_rpy)
                    
                    init_pose_joints = [(26.31/360)*tau, (-21.96/360)*tau, (106.63/360)*tau, (-90.49/360)*tau, (113.82/360)*tau, (87.36/360)*tau]          # tau = 2 * pi
                    indy10.go_to_joint_abs(init_pose_joints)

                    init_pose_joints = [(20.40/360)*tau, (6.30/360)*tau, (127.09/360)*tau, (-76.32/360)*tau, (101.79/360)*tau, (137.31/360)*tau]          # tau = 2 * pi
                    indy10.go_to_joint_abs(init_pose_joints)
                    
                    ## 공통부 1 끝
                    if warning_received == "0" and drink_info_received == "Peach":
                        # 분기 1

                        # 첫번째 시럽 위치로 이동
                        init_pose_joints = [(-11.01/360)*tau, 
                                            (-8.62/360)*tau, 
                                            (142.90/360)*tau, 
                                            (-94.37/360)*tau, 
                                            (78.57/360)*tau, 
                                            (135.54/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_on()

                        # 뒤로 감 
                        rel_xyz = [-0.1, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        rel_xyz = [0.0, 0.0, 0.3]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        indy10.gripper.grip_off()

                        # 첫번째 시헙통 누르는 좌표 (전환)
                        init_pose_joints = [(-47.07/360)*tau, 
                                            (-11.46/360)*tau, 
                                            (102.04/360)*tau, 
                                            (-45.23/360)*tau, 
                                            (52.07/360) *tau, 
                                            (61.71/360) *tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)     
                        

                        # 시럽통 누름 (고정)
                        rel_xyz = [0.0, 0.0, -0.04]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)     

                        rel_xyz = [0.0, 0.0, 0.04]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)   

                        rel_xyz = [0.0, 0.0, -0.04]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)     

                        rel_xyz = [0.0, 0.0, 0.04]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)    

                        rel_xyz = [-0.2, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        indy10.gripper.grip_on()

                        # 첫번째 시럽 담은 뒤 컵 잡으러 이동
                        init_pose_joints = [(-14.61/360)*tau, 
                                            
                                            (-12.76/360)*tau, 
                                            (140.34/360)*tau, 
                                            (-90.56/360)*tau, 
                                            (71.71/360)*tau, 
                                            (127.67/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [-0.01, 0.0, 0.005]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        indy10.gripper.grip_off()         

                        # 컵 잡은뒤 조금 들어올림
                        init_pose_joints = [(-12.79/360)*tau, 
                                            (-13.24/360)*tau, 
                                            (140.30/360)*tau, 
                                            (-90.50/360)*tau, 
                                            (73.98/360)*tau, 
                                            (128.16/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [-0.1, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        # 시럽 과정에서 중앙 위치
                        init_pose_joints = [(20.40/360)*tau, (6.30/360)*tau, (127.09/360)*tau, (-76.32/360)*tau, (101.79/360)*tau, (137.31/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        ## 공통부 2 시작
                        # init_pose_joints = [(40.92/360)*tau, (1.85/360)*tau, (83.90/360)*tau, (-91.30/360)*tau, (128.58/360)*tau, (88.14/360)*tau]          # tau = 2 * pi
                        # indy10.go_to_joint_abs(init_pose_joints)    

                        ###################################################################
                        input("============ 옯김 전단계 ...")

                        init_pose_joints = [(-18.12/360)*tau, (-12.36/360)*tau, (94.81/360)*tau, (-84.64/360)*tau, (72.10/360)*tau, (81.40/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)    

                        input("============ 옯김 전단계 ...")

                        init_pose_joints = [(-83.74/360)*tau, (-12.37/360)*tau, (101.15/360)*tau, (-90.16/360)*tau, (-81.99/360)*tau, (95.25/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)   

                        ###################################################################

                        # input("============ Press `Enter` to ice dispencer part...")

                        # 컵을 얼음 밑에 가져다 놓기
                        init_pose_joints = [(-57.13/360)*tau, 
                                            (-19.51/360)*tau, 
                                            (139.36/360)*tau, 
                                            (-74.62/360)*tau, 
                                            (-61.62/360)*tau, 
                                            (58.61/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        # input("============ Press `Enter` to ice dispencer part...")

                        rel_xyz = [0.0, 0.16, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        init_pose_joints = [(-30.56/360)*tau, 
                                            (5.27/360)*tau, 
                                            (117.89/360)*tau, 
                                            (-50.13/360)*tau, 
                                            (-42.94/360)*tau, 
                                            (41.74/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [0.0, 0.0, -0.01]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        # 컵 놓기
                        indy10.gripper.grip_on()

                        # 뒤로
                        rel_xyz = [0.0, -0.2, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)


                        rel_xyz = [0.0, 0.0, 0.2]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        # 디스펜서 돌리기
                        init_pose_joints = [(-28.06/360)*tau, 
                                            (-4.92/360)*tau, 
                                            (109.99/360)*tau, 
                                            (-64.94/360)*tau, 
                                            (-28.69/360)*tau, 
                                            (60.68/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_off()

                        init_pose_joints = [(-28.06/360)*tau, 
                                            (-4.92/360)*tau, 
                                            (109.99/360)*tau, 
                                            (-64.94/360)*tau, 
                                            (-28.69/360)*tau, 
                                            (-119.32/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        init_pose_joints = [(-28.06/360)*tau, 
                                            (-4.92/360)*tau, 
                                            (109.99/360)*tau, 
                                            (-64.94/360)*tau, 
                                            (-28.69/360)*tau, 
                                            (60.68/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_on()

                        rel_xyz = [0.0, -0.2, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        rel_xyz = [0.0, 0.0, -0.2]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        #음 담고 다시 컵 잡음

                        indy10.gripper.grip_on()
                        
                        init_pose_joints = [(-57.13/360)*tau, 
                                            (-19.51/360)*tau, 
                                            (139.36/360)*tau, 
                                            (-74.62/360)*tau, 
                                            (-61.62/360)*tau, 
                                            (58.61/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [0.0, 0.16, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        init_pose_joints = [(-30.56/360)*tau, 
                                            (5.27/360)*tau, 
                                            (117.89/360)*tau, 
                                            (-50.13/360)*tau, 
                                            (-42.94/360)*tau, 
                                            (41.74/360)*tau]         
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_off()

                        rel_xyz = [0.0, -0.3, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)       

                        input("============ 옯김 전단계 ...")

                        indy10.gripper.grip_off()

                        # input("============ Press `Enter` to ice dispencer part...")
                        # 물 펌프 이동 과정
                        init_pose_joints = [(-3.72/360)*tau, (-26.16/360)*tau, (143.63/360)*tau, (-92.24/360)*tau, (-92.92/360)*tau, (64.60/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        input("============ Press `Enter` to ice dispencer part...")
                        rel_xyz = [0.0, 0.17, 0.24]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        input("============ Press `Enter` to ice dispencer part...")

                        init_pose_joints = [(82.41/360)*tau, (-23.96/360)*tau, (116.70/360)*tau, (-89.00/360)*tau, (80.52/360)*tau, (89.20/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [0.0, 0.0, -0.15]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        # print("============ Press `Enter` to water dispencer part...")

                        init_pose_joints = [(27.96/360)*tau, (-24.25/360)*tau, (142.63/360)*tau, (-126.75/360)*tau, (38.00/360)*tau, (134.63/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        #초음파 인식 거리까지 접근
                        rel_xyz = [0.0, 0.12, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)                 

                        time.sleep(50)

                        # 다시 뒤로
                        rel_xyz = [0.0, -0.15, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)        

                        #제공하는데까지 이동
                        rel_xyz = [-0.42, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)            

                        rel_xyz = [0.0, 0.0, 0.08]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)    

                        init_pose_joints = [(45.33/360)*tau, (36.13/360)*tau, (73.93/360)*tau, (-113.40/360)*tau, (51.25/360)*tau, (122.71/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)
                        
                        rel_xyz = [0.0, 0.0, -0.02]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)    

                        # 음료수 배달
                        indy10.gripper.grip_on()

                        rel_xyz = [0.0, -0.4, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        indy10.move_to_standby()   
                        break    
                        ## 공통부 2 끝

                        ## Case 2: Orange
                    elif warning_received == "0" and drink_info_received == "Orange":

                        # Image 1
                        # new_window = f"Window_{0}"
                        # if current_window is not None and current_window != new_window:
                        #     cv2.destroyWindow(current_window)
                        # image_path = image_files[0]
                        # if os.path.exists(image_path):
                        #     image = cv2.imread(image_path)
                        #     cv2.imshow(new_window, image)
                        #     current_window = new_window 

                        ## 공통부 1
                        input("============ Moving initial pose using a joint state goal ...")

                        indy10.move_to_standby()

                        rel_xyz = [-0.16, -0.20, 0.16]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)
                        
                        indy10.gripper.grip_on()

                        init_pose_joints = [(77.00/360)*tau, (13.72/360)*tau, (100.16/360)*tau, (-94.92/360)*tau, (74.55/360)*tau, (115.63/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)
                        
                        # 컵 위치로 이동
                        init_pose_joints = [(45.04/360)*tau, (36.04/360)*tau, (71.38/360)*tau, (-103.53/360)*tau, (46.83/360)*tau, (113.89/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_width(110)

                        
                        rel_xyz = [0.0, -0.35, 0.25]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)
                        
                        init_pose_joints = [(26.31/360)*tau, (-21.96/360)*tau, (106.63/360)*tau, (-90.49/360)*tau, (113.82/360)*tau, (87.36/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        init_pose_joints = [(20.40/360)*tau, (6.30/360)*tau, (127.09/360)*tau, (-76.32/360)*tau, (101.79/360)*tau, (137.31/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)
                        
                        ## 공통부 1 끝   
                        init_pose_joints = [(-7.46/360)*tau, (0.18/360)*tau, (136.15/360)*tau, (-94.71/360)*tau, (80.31/360)*tau, (139.40/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)


                        input("------= 시럽통2 위치로 이동 -------")
                        
                        indy10.gripper.grip_on()

                        # 뒤로 감
                        rel_xyz = [-0.1, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        rel_xyz = [0.0, 0.0, 0.3]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        indy10.gripper.grip_off() 

                        # 2번째 시럽통 누르는 위치로 이동  
                        init_pose_joints = [(-39.03/360)*tau, 
                                            (-5.25/360)*tau, 
                                            (97.64/360)*tau, 
                                            (-49.83/360)*tau, 
                                            (56.72/360)*tau, 
                                            (70.33/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)        

                        # 시럽통 누르기 (공통)
                        rel_xyz = [0.0, 0.0, -0.03]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)     

                        rel_xyz = [0.0, 0.0, 0.03]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        rel_xyz = [0.0, 0.0, -0.03]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)     

                        rel_xyz = [0.0, 0.0, 0.03]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)      

                        rel_xyz = [-0.2, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        indy10.gripper.grip_on()

                        # 2번째 시럽통에 위치한 컵 잡기
                        init_pose_joints = [(-8.54/360)*tau, 
                                            (-5.58/360)*tau, 
                                            (131.79/360)*tau, 
                                            (-85.09/360)*tau, 
                                            (74.95/360)*tau, 
                                            (125.57/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [0.0, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 
                        
                        indy10.gripper.grip_off()

                        # 컵 잡은뒤 조금 들어올림
                        init_pose_joints = [(-8.56/360)*tau, 
                                            (-6.00/360)*tau, 
                                            (131.74/360)*tau, 
                                            (-85.42/360)*tau, 
                                            (75.28/360)*tau, 
                                            (125.89/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [-0.1, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        # 시럽 과정에서 중앙 위치
                        init_pose_joints = [(20.40/360)*tau, (6.30/360)*tau, (127.09/360)*tau, (-76.32/360)*tau, (101.79/360)*tau, (137.31/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)             

                        init_pose_joints = [(40.92/360)*tau, (1.85/360)*tau, (83.90/360)*tau, (-91.30/360)*tau, (128.58/360)*tau, (88.14/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)    

                        ###################################################################
                        input("============ 옯김 전단계 ...")

                        init_pose_joints = [(-18.12/360)*tau, (-12.36/360)*tau, (94.81/360)*tau, (-84.64/360)*tau, (72.10/360)*tau, (81.40/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)    

                        input("============ 옯김 전단계 ...")

                        init_pose_joints = [(-83.74/360)*tau, (-12.37/360)*tau, (101.15/360)*tau, (-90.16/360)*tau, (-81.99/360)*tau, (95.25/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)   

                        ###################################################################

                        # input("============ Press `Enter` to ice dispencer part...")

                        # 컵을 얼음 밑에 가져다 놓기
                        init_pose_joints = [(-57.13/360)*tau, 
                                            (-19.51/360)*tau, 
                                            (139.36/360)*tau, 
                                            (-74.62/360)*tau, 
                                            (-61.62/360)*tau, 
                                            (58.61/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        # input("============ Press `Enter` to ice dispencer part...")

                        rel_xyz = [0.0, 0.16, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        init_pose_joints = [(-30.56/360)*tau, 
                                            (5.27/360)*tau, 
                                            (117.89/360)*tau, 
                                            (-50.13/360)*tau, 
                                            (-42.94/360)*tau, 
                                            (41.74/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [0.0, 0.0, -0.01]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        # 컵 놓기
                        indy10.gripper.grip_on()

                        # 뒤로
                        rel_xyz = [0.0, -0.2, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)


                        rel_xyz = [0.0, 0.0, 0.2]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        # 디스펜서 돌리기
                        init_pose_joints = [(-28.06/360)*tau, 
                                            (-4.92/360)*tau, 
                                            (109.99/360)*tau, 
                                            (-64.94/360)*tau, 
                                            (-28.69/360)*tau, 
                                            (60.68/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_off()

                        init_pose_joints = [(-28.06/360)*tau, 
                                            (-4.92/360)*tau, 
                                            (109.99/360)*tau, 
                                            (-64.94/360)*tau, 
                                            (-28.69/360)*tau, 
                                            (-119.32/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        init_pose_joints = [(-28.06/360)*tau, 
                                            (-4.92/360)*tau, 
                                            (109.99/360)*tau, 
                                            (-64.94/360)*tau, 
                                            (-28.69/360)*tau, 
                                            (60.68/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_on()

                        rel_xyz = [0.0, -0.2, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        rel_xyz = [0.0, 0.0, -0.2]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        #음 담고 다시 컵 잡음

                        indy10.gripper.grip_on()
                        
                        init_pose_joints = [(-57.13/360)*tau, 
                                            (-19.51/360)*tau, 
                                            (139.36/360)*tau, 
                                            (-74.62/360)*tau, 
                                            (-61.62/360)*tau, 
                                            (58.61/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [0.0, 0.16, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        init_pose_joints = [(-30.56/360)*tau, 
                                            (5.27/360)*tau, 
                                            (117.89/360)*tau, 
                                            (-50.13/360)*tau, 
                                            (-42.94/360)*tau, 
                                            (41.74/360)*tau]         
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_off()

                        rel_xyz = [0.0, -0.3, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)       

                        input("============ 옯김 전단계 ...")

                        indy10.gripper.grip_off()

                        # input("============ Press `Enter` to ice dispencer part...")
                        # 물 펌프 이동 과정
                        init_pose_joints = [(-3.72/360)*tau, (-26.16/360)*tau, (143.63/360)*tau, (-92.24/360)*tau, (-92.92/360)*tau, (64.60/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        input("============ Press `Enter` to ice dispencer part...")
                        rel_xyz = [0.0, 0.17, 0.24]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        input("============ Press `Enter` to ice dispencer part...")

                        init_pose_joints = [(82.41/360)*tau, (-23.96/360)*tau, (116.70/360)*tau, (-89.00/360)*tau, (80.52/360)*tau, (89.20/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [0.0, 0.0, -0.15]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        # print("============ Press `Enter` to water dispencer part...")

                        init_pose_joints = [(27.96/360)*tau, (-24.25/360)*tau, (142.63/360)*tau, (-126.75/360)*tau, (38.00/360)*tau, (134.63/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        #초음파 인식 거리까지 접근
                        rel_xyz = [0.0, 0.12, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)                 

                        time.sleep(50)

                        # 다시 뒤로
                        rel_xyz = [0.0, -0.15, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)        

                        #제공하는데까지 이동
                        rel_xyz = [-0.42, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)            

                        rel_xyz = [0.0, 0.0, 0.08]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)    

                        init_pose_joints = [(45.33/360)*tau, (36.13/360)*tau, (73.93/360)*tau, (-113.40/360)*tau, (51.25/360)*tau, (122.71/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)
                        
                        rel_xyz = [0.0, 0.0, -0.02]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)    

                        # 음료수 배달
                        indy10.gripper.grip_on()

                        rel_xyz = [0.0, -0.4, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        indy10.move_to_standby()   
                        break

                        # Case 3: Strawberry
                    elif warning_received == "0" and drink_info_received == "Strawberry":
                        # Image 1
                        # new_window = f"Window_{0}"
                        # if current_window is not None and current_window != new_window:
                        #     cv2.destroyWindow(current_window)
                        # image_path = image_files[0]
                        # if os.path.exists(image_path):
                        #     image = cv2.imread(image_path)
                        #     cv2.imshow(new_window, image)
                        #     current_window = new_window 
                        ## 공통부 1
                        input("============ Moving initial pose using a joint state goal ...")

                        indy10.move_to_standby()

                        rel_xyz = [-0.16, -0.20, 0.16]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)
                        
                        indy10.gripper.grip_on()

                        init_pose_joints = [(77.00/360)*tau, (13.72/360)*tau, (100.16/360)*tau, (-94.92/360)*tau, (74.55/360)*tau, (115.63/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)
                        
                        # 컵 위치로 이동
                        init_pose_joints = [(45.04/360)*tau, (36.04/360)*tau, (71.38/360)*tau, (-103.53/360)*tau, (46.83/360)*tau, (113.89/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_width(110)

                        
                        rel_xyz = [0.0, -0.35, 0.25]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)
                        
                        init_pose_joints = [(26.31/360)*tau, (-21.96/360)*tau, (106.63/360)*tau, (-90.49/360)*tau, (113.82/360)*tau, (87.36/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        init_pose_joints = [(20.40/360)*tau, (6.30/360)*tau, (127.09/360)*tau, (-76.32/360)*tau, (101.79/360)*tau, (137.31/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)
                        
                        ## 공통부 1 끝   
                        init_pose_joints = [(-7.46/360)*tau, (0.18/360)*tau, (136.15/360)*tau, (-94.71/360)*tau, (80.31/360)*tau, (139.40/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)


                        input("------= 시럽통3 위치로 이동 -------")

                        init_pose_joints = [(-6.25/360)*tau, 
                                            (5.91/360)*tau, 
                                            (124.86/360)*tau, 
                                            (-89.53/360)*tau, 
                                            (78.87/360)*tau, 
                                            (130.88/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)


                        input("------= 시럽통3 위치로 이동 -------")
                        
                        indy10.gripper.grip_on()

                        # 뒤로 감
                        rel_xyz = [-0.1, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        rel_xyz = [0.0, 0.0, 0.3]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        indy10.gripper.grip_off() 

                        #세번째 누르는 좌표
                        init_pose_joints = [(-33.31/360)*tau, 
                                            (3.59/360)*tau, 
                                            (88.71/360)*tau, 
                                            (-51.86/360)*tau, 
                                            (60.28/360)*tau, 
                                            (74.26/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)        

                        input("============ Press `Enter` to press...")
                        rel_xyz = [0.0, 0.0, -0.03]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)     

                        rel_xyz = [0.0, 0.0, 0.04]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)  

                        rel_xyz = [0.0, 0.0, -0.03]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)     

                        rel_xyz = [0.0, 0.0, 0.04]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)  

                        rel_xyz = [-0.2, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        indy10.gripper.grip_on()

                        # 3번째 시럽통에 위치한 컵 잡기
                        init_pose_joints = [(-11.34/360)*tau, 
                                            (5.09/360)*tau, 
                                            (122.46/360)*tau, 
                                            (-88.30/360)*tau, 
                                            (72.60/360)*tau, 
                                            (128.64/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        input("============ Press `Enter` to press...")

                        rel_xyz = [-0.02, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 
                        
                        indy10.gripper.grip_off()

                        input("============ Press `Enter` to press...")

                        # 컵 잡은뒤 조금 들어올림
                        init_pose_joints = [(-7.76/360)*tau, 
                                            (3.67/360)*tau, 
                                            (122.84/360)*tau, 
                                            (-86.94/360)*tau, 
                                            (75.54/360)*tau, 
                                            (127.10/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [-0.1, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        # 시럽 과정에서 중앙 위치
                        init_pose_joints = [(20.40/360)*tau, (6.30/360)*tau, (127.09/360)*tau, (-76.32/360)*tau, (101.79/360)*tau, (137.31/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        ######################################################################
                

                        init_pose_joints = [(40.92/360)*tau, (1.85/360)*tau, (83.90/360)*tau, (-91.30/360)*tau, (128.58/360)*tau, (88.14/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)    

                        ###################################################################
                        input("============ 옯김 전단계 ...")

                        init_pose_joints = [(-18.12/360)*tau, (-12.36/360)*tau, (94.81/360)*tau, (-84.64/360)*tau, (72.10/360)*tau, (81.40/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)    

                        input("============ 옯김 전단계 ...")

                        init_pose_joints = [(-83.74/360)*tau, (-12.37/360)*tau, (101.15/360)*tau, (-90.16/360)*tau, (-81.99/360)*tau, (95.25/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)   

                        ###################################################################

                        # input("============ Press `Enter` to ice dispencer part...")

                        # 컵을 얼음 밑에 가져다 놓기
                        init_pose_joints = [(-57.13/360)*tau, 
                                            (-19.51/360)*tau, 
                                            (139.36/360)*tau, 
                                            (-74.62/360)*tau, 
                                            (-61.62/360)*tau, 
                                            (58.61/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        # input("============ Press `Enter` to ice dispencer part...")

                        rel_xyz = [0.0, 0.16, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        init_pose_joints = [(-30.56/360)*tau, 
                                            (5.27/360)*tau, 
                                            (117.89/360)*tau, 
                                            (-50.13/360)*tau, 
                                            (-42.94/360)*tau, 
                                            (41.74/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [0.0, 0.0, -0.01]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        # 컵 놓기
                        indy10.gripper.grip_on()

                        # 뒤로
                        rel_xyz = [0.0, -0.2, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)


                        rel_xyz = [0.0, 0.0, 0.2]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        # 디스펜서 돌리기
                        init_pose_joints = [(-28.06/360)*tau, 
                                            (-4.92/360)*tau, 
                                            (109.99/360)*tau, 
                                            (-64.94/360)*tau, 
                                            (-28.69/360)*tau, 
                                            (60.68/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_off()

                        init_pose_joints = [(-28.06/360)*tau, 
                                            (-4.92/360)*tau, 
                                            (109.99/360)*tau, 
                                            (-64.94/360)*tau, 
                                            (-28.69/360)*tau, 
                                            (-119.32/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        init_pose_joints = [(-28.06/360)*tau, 
                                            (-4.92/360)*tau, 
                                            (109.99/360)*tau, 
                                            (-64.94/360)*tau, 
                                            (-28.69/360)*tau, 
                                            (60.68/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_on()

                        rel_xyz = [0.0, -0.2, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        rel_xyz = [0.0, 0.0, -0.2]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        #음 담고 다시 컵 잡음

                        indy10.gripper.grip_on()
                        
                        init_pose_joints = [(-57.13/360)*tau, 
                                            (-19.51/360)*tau, 
                                            (139.36/360)*tau, 
                                            (-74.62/360)*tau, 
                                            (-61.62/360)*tau, 
                                            (58.61/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [0.0, 0.16, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        init_pose_joints = [(-30.56/360)*tau, 
                                            (5.27/360)*tau, 
                                            (117.89/360)*tau, 
                                            (-50.13/360)*tau, 
                                            (-42.94/360)*tau, 
                                            (41.74/360)*tau]         
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_off()

                        rel_xyz = [0.0, -0.3, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)       

                        input("============ 옯김 전단계 ...")

                        indy10.gripper.grip_off()

                        # input("============ Press `Enter` to ice dispencer part...")
                        # 물 펌프 이동 과정
                        init_pose_joints = [(-3.72/360)*tau, (-26.16/360)*tau, (143.63/360)*tau, (-92.24/360)*tau, (-92.92/360)*tau, (64.60/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        input("============ Press `Enter` to ice dispencer part...")
                        rel_xyz = [0.0, 0.17, 0.24]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        input("============ Press `Enter` to ice dispencer part...")

                        init_pose_joints = [(82.41/360)*tau, (-23.96/360)*tau, (116.70/360)*tau, (-89.00/360)*tau, (80.52/360)*tau, (89.20/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [0.0, 0.0, -0.15]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        # print("============ Press `Enter` to water dispencer part...")

                        init_pose_joints = [(27.96/360)*tau, (-24.25/360)*tau, (142.63/360)*tau, (-126.75/360)*tau, (38.00/360)*tau, (134.63/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        #초음파 인식 거리까지 접근
                        rel_xyz = [0.0, 0.12, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)                 

                        time.sleep(50)

                        # 다시 뒤로
                        rel_xyz = [0.0, -0.15, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)        

                        #제공하는데까지 이동
                        rel_xyz = [-0.42, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)            

                        rel_xyz = [0.0, 0.0, 0.08]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)    

                        init_pose_joints = [(45.33/360)*tau, (36.13/360)*tau, (73.93/360)*tau, (-113.40/360)*tau, (51.25/360)*tau, (122.71/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)
                        
                        rel_xyz = [0.0, 0.0, -0.02]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)    

                        # 음료수 배달
                        indy10.gripper.grip_on()

                        rel_xyz = [0.0, -0.4, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        indy10.move_to_standby()   
                        break

                        # Case 4: Peach
                    elif warning_received == "0" and drink_info_received == "Peach":
                        # Image 1
                        # new_window = f"Window_{0}"
                        # if current_window is not None and current_window != new_window:
                        #     cv2.destroyWindow(current_window)
                        # image_path = image_files[0]
                        # if os.path.exists(image_path):
                        #     image = cv2.imread(image_path)
                        #     cv2.imshow(new_window, image)
                        #     current_window = new_window 
                        ## 공통부 1
                        input("============ Moving initial pose using a joint state goal ...")

                        indy10.move_to_standby()

                        rel_xyz = [-0.16, -0.20, 0.16]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)
                        
                        indy10.gripper.grip_on()

                        init_pose_joints = [(77.00/360)*tau, (13.72/360)*tau, (100.16/360)*tau, (-94.92/360)*tau, (74.55/360)*tau, (115.63/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)
                        
                        # 컵 위치로 이동
                        init_pose_joints = [(45.04/360)*tau, (36.04/360)*tau, (71.38/360)*tau, (-103.53/360)*tau, (46.83/360)*tau, (113.89/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_width(110)

                        
                        rel_xyz = [0.0, -0.35, 0.25]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)
                        
                        init_pose_joints = [(26.31/360)*tau, (-21.96/360)*tau, (106.63/360)*tau, (-90.49/360)*tau, (113.82/360)*tau, (87.36/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        init_pose_joints = [(20.40/360)*tau, (6.30/360)*tau, (127.09/360)*tau, (-76.32/360)*tau, (101.79/360)*tau, (137.31/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)
                        
                        ## 공통부 1 끝   
                        print("------= 시럽통4 위치로 이동 -------")

                        init_pose_joints = [(-7.19/360)*tau, 
                                            (17.75/360)*tau, 
                                            (116.36/360)*tau, 
                                            (-94.81/360)*tau, 
                                            (81.37/360)*tau, 
                                            (137.34/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)


                        print("------= 시럽통4 위치로 이동 -------")
                        
                        indy10.gripper.grip_on()

                        # 뒤로 감
                        rel_xyz = [-0.1, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        rel_xyz = [0.0, 0.0, 0.3]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        indy10.gripper.grip_off() 

                        #네번째 누르는 코드
                        init_pose_joints = [(-29.16/360)*tau, 
                                            (8.31/360)*tau, 
                                            (88.22/360)*tau, 
                                            (-60.98/360)*tau, 
                                            (56.01/360)*tau, 
                                            (93.50/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        print("============ Press `Enter` to press...")
                        rel_xyz = [0.0, 0.0, -0.03]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)     

                        rel_xyz = [0.0, 0.0, 0.04]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)  

                        rel_xyz = [0.0, 0.0, -0.03]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)     

                        rel_xyz = [0.0, 0.0, 0.04]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        rel_xyz = [-0.2, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        indy10.gripper.grip_on()

                        # 3번째 시럽통에 위치한 컵 잡기
                        init_pose_joints = [(-10.15/360)*tau, 
                                            (15.96/360)*tau, 
                                            (114.41/360)*tau, 
                                            (-92.34/360)*tau, 
                                            (76.43/360)*tau, 
                                            (133.12/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        print("============ Press `Enter` to press...")

                        rel_xyz = [-0.01, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 
                        
                        indy10.gripper.grip_off()

                        print("============ Press `Enter` to press...")

                        # 컵 잡은뒤 조금 들어올림
                        init_pose_joints = [(-5.79/360)*tau, 
                                            (17.55/360)*tau, 
                                            (114.31/360)*tau, 
                                            (-92.30/360)*tau, 
                                            (80.87/360)*tau, 
                                            (130.66/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [-0.1, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        # 시럽 과정에서 중앙 위치
                        init_pose_joints = [(20.40/360)*tau, (6.30/360)*tau, (127.09/360)*tau, (-76.32/360)*tau, (101.79/360)*tau, (137.31/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)      

                        ######################################################################
                

                        init_pose_joints = [(40.92/360)*tau, (1.85/360)*tau, (83.90/360)*tau, (-91.30/360)*tau, (128.58/360)*tau, (88.14/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)    

                        ###################################################################
                        input("============ 옯김 전단계 ...")

                        init_pose_joints = [(-18.12/360)*tau, (-12.36/360)*tau, (94.81/360)*tau, (-84.64/360)*tau, (72.10/360)*tau, (81.40/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)    

                        input("============ 옯김 전단계 ...")

                        init_pose_joints = [(-83.74/360)*tau, (-12.37/360)*tau, (101.15/360)*tau, (-90.16/360)*tau, (-81.99/360)*tau, (95.25/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)   

                        ###################################################################

                        # input("============ Press `Enter` to ice dispencer part...")

                        # 컵을 얼음 밑에 가져다 놓기
                        init_pose_joints = [(-57.13/360)*tau, 
                                            (-19.51/360)*tau, 
                                            (139.36/360)*tau, 
                                            (-74.62/360)*tau, 
                                            (-61.62/360)*tau, 
                                            (58.61/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        # input("============ Press `Enter` to ice dispencer part...")

                        rel_xyz = [0.0, 0.16, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        init_pose_joints = [(-30.56/360)*tau, 
                                            (5.27/360)*tau, 
                                            (117.89/360)*tau, 
                                            (-50.13/360)*tau, 
                                            (-42.94/360)*tau, 
                                            (41.74/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [0.0, 0.0, -0.01]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        # 컵 놓기
                        indy10.gripper.grip_on()

                        # 뒤로
                        rel_xyz = [0.0, -0.2, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)


                        rel_xyz = [0.0, 0.0, 0.2]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        # 디스펜서 돌리기
                        init_pose_joints = [(-28.06/360)*tau, 
                                            (-4.92/360)*tau, 
                                            (109.99/360)*tau, 
                                            (-64.94/360)*tau, 
                                            (-28.69/360)*tau, 
                                            (60.68/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_off()

                        init_pose_joints = [(-28.06/360)*tau, 
                                            (-4.92/360)*tau, 
                                            (109.99/360)*tau, 
                                            (-64.94/360)*tau, 
                                            (-28.69/360)*tau, 
                                            (-119.32/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        init_pose_joints = [(-28.06/360)*tau, 
                                            (-4.92/360)*tau, 
                                            (109.99/360)*tau, 
                                            (-64.94/360)*tau, 
                                            (-28.69/360)*tau, 
                                            (60.68/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_on()

                        rel_xyz = [0.0, -0.2, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        rel_xyz = [0.0, 0.0, -0.2]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        #음 담고 다시 컵 잡음

                        indy10.gripper.grip_on()
                        
                        init_pose_joints = [(-57.13/360)*tau, 
                                            (-19.51/360)*tau, 
                                            (139.36/360)*tau, 
                                            (-74.62/360)*tau, 
                                            (-61.62/360)*tau, 
                                            (58.61/360)*tau]          
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [0.0, 0.16, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        init_pose_joints = [(-30.56/360)*tau, 
                                            (5.27/360)*tau, 
                                            (117.89/360)*tau, 
                                            (-50.13/360)*tau, 
                                            (-42.94/360)*tau, 
                                            (41.74/360)*tau]         
                        indy10.go_to_joint_abs(init_pose_joints)

                        indy10.gripper.grip_off()

                        rel_xyz = [0.0, -0.3, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)       

                        input("============ 옯김 전단계 ...")

                        indy10.gripper.grip_off()

                        # input("============ Press `Enter` to ice dispencer part...")
                        # 물 펌프 이동 과정
                        init_pose_joints = [(-3.72/360)*tau, (-26.16/360)*tau, (143.63/360)*tau, (-92.24/360)*tau, (-92.92/360)*tau, (64.60/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        input("============ Press `Enter` to ice dispencer part...")
                        rel_xyz = [0.0, 0.17, 0.24]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        input("============ Press `Enter` to ice dispencer part...")

                        init_pose_joints = [(82.41/360)*tau, (-23.96/360)*tau, (116.70/360)*tau, (-89.00/360)*tau, (80.52/360)*tau, (89.20/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        rel_xyz = [0.0, 0.0, -0.15]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)

                        # print("============ Press `Enter` to water dispencer part...")

                        init_pose_joints = [(27.96/360)*tau, (-24.25/360)*tau, (142.63/360)*tau, (-126.75/360)*tau, (38.00/360)*tau, (134.63/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)

                        #초음파 인식 거리까지 접근
                        rel_xyz = [0.0, 0.12, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)                 

                        time.sleep(50)

                        # 다시 뒤로
                        rel_xyz = [0.0, -0.15, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)        

                        #제공하는데까지 이동
                        rel_xyz = [-0.42, 0.0, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)            

                        rel_xyz = [0.0, 0.0, 0.08]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)    

                        init_pose_joints = [(45.33/360)*tau, (36.13/360)*tau, (73.93/360)*tau, (-113.40/360)*tau, (51.25/360)*tau, (122.71/360)*tau]          # tau = 2 * pi
                        indy10.go_to_joint_abs(init_pose_joints)
                        
                        rel_xyz = [0.0, 0.0, -0.02]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy)    

                        # 음료수 배달
                        indy10.gripper.grip_on()

                        rel_xyz = [0.0, -0.4, 0.0]
                        rel_rpy = [0.0, 0.0, 0.0]
                        indy10.go_to_pose_rel(rel_xyz, rel_rpy) 

                        indy10.move_to_standby()   
                        break
                    

                    # Warnings
                    elif warning_received == "1":
                        print("Green Grape Syrup Need to Change!")
                        display_image("/home/sionseo/catkin_ws/src/indy_driver/src/green_grape_change.png")
                        time.sleep(10)
                        break           
                    elif warning_received == "2":
                        print("Orange Syrup Need to Change!")
                        display_image("/home/sionseo/catkin_ws/src/indy_driver/src/orange_change.png")
                        time.sleep(10)
                        break
                    elif warning_received == "3":
                        print("Strawberry Syrup Need to Change!")
                        display_image("/home/sionseo/catkin_ws/src/indy_driver/src/strawberry_change.png")
                        time.sleep(10)
                        break                  
                    elif warning_received == "4":
                        print("Peach Syrup Need to Change!")
                        display_image("/home/sionseo/catkin_ws/src/indy_driver/src/peach_change.png")
                        time.sleep(10)
                        break       
  
         
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":      
    main()
