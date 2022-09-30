#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import sys,os
import rospy
import rospkg
import numpy as np
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import tf
from math import cos,sin,sqrt,pow,atan2,pi,ceil
from tf.transformations import quaternion_from_euler
from lib.utils import pathReader, findLocalPath

class MoveBaseFollower:
    def __init__(self):
        #init node 
        rospy.init_node('movebase_follower', anonymous=True)
        self.configure()
        self.is_status=False ## 차량 상태 점검

        # Path data reader
        path_reader = pathReader('path_extraction') ## 경로 파일의 패키지 위치
        self.global_path = path_reader.read_txt(self.path_file_name, self.path_frame) ## 출력할 경로의 이름

        # ROS publsiher & subscriber 
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseGoal, queue_size=1) ## Vehicle Control
        rospy.Subscriber("/map2gps_pose", Odometry, self.statusCB)

        # time var
        self.rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():
            
            if self.is_status:
                self.send_goal()

            self.rate.sleep()

    def configure(self):
        self.path_file_name = rospy.get_param("~path_file_name")
        self.path_frame = rospy.get_param("~path_frame")
        self.frequency = rospy.get_param("~frequency")
        self.lai = rospy.get_param("~look_ahead_index")
        self.lad = rospy.get_param("~look_ahead_distance")
        self.point_interval = rospy.get_param("~~min_sample_distance")

    def statusCB(self, msg): ## Vehicle Status Subscriber 
        self.is_status=True
        self.status_msg = msg


    def send_goal(self):
        ## Local path중 look ahead distance 앞에 있는 점을 move base goal로 만듬  
        vehicle_pose = self.status_msg.pose.pose.postion
        self.local_path, self.current_waypoint = findLocalPath(self.global_path, self.status_msg, self.path_frame, self.lai)
        
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

if __name__ == '__main__':
    MoveBaseFollower()

