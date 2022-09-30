#!/usr/bin/env python

print('file init')

import os
import sys
import rospy
import rospkg

from math import pi,cos,sin,pi,sqrt,pow
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from math import cos,sin,sqrt,pow,atan2,pi
import tf

class PathExtraction :

    def __init__(self):
        rospy.init_node('path_extraction', anonymous=True)

        print('ros init')
        # ROS Publisher
        self.path_rviz_pub = rospy.Publisher("/path_rviz", Path, queue_size=10)
        self.path_msg = Path()
        
        # ROS Subscriber
        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.gpsCallback)
        rospy.Subscriber("/map2gps_pose", Odometry, self.poseCallback)

        print('configure start')
        self.configure()
        print('configure end')
        self.is_status = False
        self.prev_x = 0
        self.prev_y = 0
        self.gps_init_latitude = 0
        self.gps_init_longitude = 0
        self.gps_first = False
        
        # find directory to save path file 
        self.touchPathfile()

        # receive pose msg and write path file
        rate=rospy.Rate(30)

        print('before loop')
        try:
            while not rospy.is_shutdown():
                if self.is_status==True :
                    self.savePath()
                rate.sleep()    
        except KeyboardInterrupt:
            self.f.close()
            rospy.logwarn("---------------------------------------------------------------------")
            rospy.loginfo("[Init GPS Origin] Latitude / Longitude : %.9f / %.9f", self.gps_init_latitude, self.gps_init_longitude)
            rospy.logwarn("---------------------------------------------------------------------")
    
    def configure(self):
        self.path_directory_name = rospy.get_param("~path_directory_name")
        self.path_file_name = rospy.get_param("~path_file_name")

        self.min_sample_distance = rospy.get_param("~min_sample_distance")

    def touchPathfile(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('path_extraction')
        full_path = pkg_path +'/'+ self.path_directory_name+'/'+self.path_file_name
        self.f = open(full_path, 'w')

    def gpsCallback(self, msg):
        if self.gps_first == False:
            self.gps_init_latitude = msg.latitude
            self.gps_init_longitude = msg.longitude
            self.gps_first = True

            rospy.logwarn("---------------------------------------------------------------------")
            rospy.loginfo("[Init GPS Origin] Latitude / Longitude : %.9f / %.9f", self.gps_init_latitude, self.gps_init_longitude)
            rospy.logwarn("---------------------------------------------------------------------")

    def poseCallback(self, msg):  # odom version (odom -> basefootprint)
        self.is_status= True

        self.path_frame_id = msg.header.frame_id # map frame
        self.status_msg= msg.pose.pose
        Ego_HeadingAngle = [self.status_msg.orientation.x, self.status_msg.orientation.y, self.status_msg.orientation.z, self.status_msg.orientation.w]

        self.TFsender = tf.TransformBroadcaster()
        self.TFsender.sendTransform((self.status_msg.position.x, self.status_msg.position.y, 0),
                        Ego_HeadingAngle,
                        rospy.Time.now(),
                        "gps", # child frame "base_link"
                        "map") # parent frame "map"

        # self.TFlistener = tf.TransformListener()
        # try:
        #     (trans,rot) = self.TFlistener.lookupTransform('/frame1', '/frame2', rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     pass

        # angular = 4 * atan2(trans[1], trans[0])
        # linear = 0.5 * sqrt(trans[0] ** 2 + trans[1] ** 2)

    def savePath(self): 
        x = self.status_msg.position.x
        y = self.status_msg.position.y
        z = 0

        # distance between 2 path points
        distance = sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))

        if distance > self.min_sample_distance:
            data='{0}\t{1}\t{2}\n'.format(x,y,z)
            self.f.write(data)
            self.prev_x=x
            self.prev_y=y

            # debug 
            print(x,y)
            
            last_point = PoseStamped()
            last_point.pose.position.x = x
            last_point.pose.position.y = y
            last_point.pose.position.z = 0
            last_point.pose.orientation.x = 0
            last_point.pose.orientation.y = 0
            last_point.pose.orientation.z = 0
            last_point.pose.orientation.w = 1

            self.path_msg.header.frame_id = self.path_frame_id
            # self.path_msg.header.frame_id = 'map'
            self.path_msg.poses.append(last_point)
            self.path_rviz_pub.publish(self.path_msg)            
        
if __name__ == '__main__':
    try:
        path_extracter = PathExtraction()
    except rospy.ROSInterruptException:
        pass