#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import sqrt
import sys,os
import rospy
import rospkg
import numpy as np
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import tf
from tf.transformations import quaternion_from_euler
from lib.utils import pathReader, findLocalPath

class trackTracker:
    def __init__(self):
        #init node 
        rospy.init_node('track_tracker', anonymous=True)
        self.configure()
        self.is_status=False ## 차량 상태 점검

        # Path data reader
        path_reader = pathReader('path_extraction') ## 경로 파일의 패키지 위치
        self.global_path = path_reader.read_txt(self.path_file_name, self.path_frame) ## 출력할 경로의 이름

        # ROS publsiher & subscriber 
        self.goal_pub = rospy.Publsiher('/move_base/goal', MoveBaseGoal, queue_size=1) ## Vehicle Control
        rospy.Subscriber("/map2gps_pose", Odometry, self.statusCB)

        # time var
        self.count = 0
        self.rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():
            if self.is_status==True:
                self.spin_once()

                if self.count == self.frequency : ## goal publish 및 global path 출력 
                    self.send_goal()
                    self.global_path_pub.publish(self.global_path)
                    self.count=0
                self.count+=1

            self.rate.sleep()

    def configure(self):
        self.path_file_name = rospy.get_param("~path_file_name")
        self.path_frame = rospy.get_param("~path_frame")
        self.frequency = rospy.get_param("~frequency")
        self.local_path_step = rospy.get_param("~local_path_step")
        self.lad = rospy.get_param("~look_ahead_distance")
        self.point_interval = rospy.get_param("~point_interval")


    def statusCB(self, msg): ## Vehicle Status Subscriber 
        self.is_status=True
        self.status_msg = msg

        Ego_HeadingAngle = [self.status_msg.pose.pose.orientation.x, self.status_msg.pose.pose.orientation.y, self.status_msg.pose.pose.orientation.z, self.status_msg.pose.pose.orientation.w]
        
        # Map -> gps TF Broadcaster
        self.TFsender = tf.TransformBroadcaster()
        self.TFsender.sendTransform((self.status_msg.pose.pose.position.x, self.status_msg.pose.pose.position.y, 0),
                        Ego_HeadingAngle,
                        rospy.Time.now(),
                        "gps", # child frame "base_link"
                        self.path_frame) # parent frame "map"

        # Odometry history viewer
        last_point = PoseStamped()
        last_point.pose.position.x = self.status_msg.pose.pose.position.x
        last_point.pose.position.y = self.status_msg.pose.pose.position.y
        last_point.pose.position.z = 0

        last_point.pose.orientation.x = self.status_msg.pose.pose.orientation.x
        last_point.pose.orientation.y = self.status_msg.pose.pose.orientation.y
        last_point.pose.orientation.z = self.status_msg.pose.pose.orientation.z
        last_point.pose.orientation.w = self.status_msg.pose.pose.orientation.w

        self.odometry_path_msg.header.frame_id = self.path_frame
        self.odometry_path_msg.poses.append(last_point)
        self.odometry_path_pub.publish(self.odometry_path_msg)  

    def spin_once(self):
        ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
        self.local_path, self.current_waypoint = findLocalPath(self.global_path, self.status_msg, self.path_frame, self.local_path_step)
        self.local_path_pub.publish(self.local_path) ## Local Path 출력

    def send_goal(self):
        ## Local path중 look ahead distance 앞에 있는 점을 move base goal로 만듬  
        vehicle_pose = self.status_msg.pose.pose.postion
        for i, point in enumerate(self.local_path.poses):
            # 차와의 거리가 look ahead point보다 작을 수 밖에 없는 점 무시 
            if (i<round(self.lad/self.point_interval)-1):
                continue
            
            dx= point.pose.position.x - vehicle_pose.x
            dy= point.pose.position.y - vehicle_pose.y
            dis = sqrt(pow(dx,2)+pow(dy,2))

            if dis >= self.lad:
                break
            
            if i == len(self.local_path.poses)-1:
                rospy.loginfo("no point out of look ahead distance")
                #local path 중 look ahead distance 보다 먼 점이 없다면 맨 마지막 포인트를 goal로 설정
                
        goal_point = point

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = goal_point.pose.position.x
        self.goal.target_pose.pose.position.y = goal_point.pose.position.y
        quaternion = tf.transformations.quaternion_from_euler(0,0,goal_point.pose.position.z)
        self.goal.target_pose.pose.orientation.x = quaternion[0]
        self.goal.target_pose.pose.orientation.y = quaternion[1]
        self.goal.target_pose.pose.orientation.z = quaternion[2]
        self.goal.target_pose.pose.orientation.w = quaternion[3]

        self.goal_pub.publish(self.goal)



