#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
from utils.utils import *
from utils.icp import icp

pose = [-81.4857, 9.0564, 0] # x, y, heading
pose = [0, 0, 0] # x, y, heading
prev_pcd = None

def callback(pcd):
    global prev_pcd, pose
    
    while pcd.data is None:
        rospy.loginfo('Lidar data is empty...')
        return None
    
    current_local_pcd = pre_processing(pcd)
 
    if prev_pcd is not None:
        H, H2d = icp(A=current_local_pcd, B=prev_pcd)
    
    else:
        H, H2d = icp(A=current_local_pcd, B=current_local_pcd)
    
    pose_T = v2t(pose)
    pose = t2v(np.dot(H2d, pose_T))
    print(pose)
    draw_pose(pose)
    
    prev_pcd = current_local_pcd

if __name__ == '__main__':
    rospy.init_node('ICP', anonymous=True)
    rospy.loginfo('ICP' + ' is now working...')
    rospy.Subscriber('/velodyne_points', PointCloud2, callback, queue_size=3)
    
    while not rospy.is_shutdown():
        rospy.spin()