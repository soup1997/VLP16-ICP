#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
from utils.utils import *
from utils.icp import icp

pose = [-81.4857, 9.0564, 0] # x, y, heading
prev_pcd = None

<<<<<<< HEAD
def callback(pcd):
    global prev_pcd, pose
    
    while pcd.data is None:
        rospy.loginfo('Lidar data is empty...')
        return None
=======
def lidar_callback(pcd):
    while pcd.data is None:
        rospy.loginfo('Lidar data is empty...')
        return
>>>>>>> 7a1e94463fcdc0f23c77d9cb0dd911554c624ead
    
    current_local_pcd = pre_processing(pcd)
 
    if prev_pcd is not None:
        H, H2d = icp(A=current_local_pcd, B=prev_pcd)
    
    else:
        H, H2d = icp(A=current_local_pcd, B=current_local_pcd)
    
    pose_T = v2t(pose)
    pose = t2v(np.dot(H2d, pose_T))
    draw_pose(pose)
    
    prev_pcd = current_local_pcd

if __name__ == '__main__':
    rospy.init_node('ICP', anonymous=True)
    rospy.loginfo('ICP' + ' is now working...')
    rospy.Subscriber('/velodyne_points', PointCloud2, lidar_callback, queue_size=3)
    
    rospy.spin()
