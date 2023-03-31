#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
from utils.utils import *
from utils.icp import icp

pose = [0, 0, 0]
odom = []
prev_pcd = None

def callback(pcd):
    global prev_pcd
    
    while pcd.data is None:
        rospy.loginfo('Lidar data is empty...')
    
    current_pcd = pre_processing(pcd)
    
    if prev_pcd is not None:
        H, A = icp(A=current_pcd, B=prev_pcd)
        
        res_pcd = np.ones((A.shape[0], 4))
        
        res_pcd[:, :3] = A
        res_pcd = np.dot(H, res_pcd.T).T
        
    prev_pcd = current_pcd
    
if __name__ == '__main__':
    rospy.init_node('ICP', anonymous=True)
    rospy.loginfo('ICP' + ' is now working...')
    rospy.Subscriber('/velodyne_points', PointCloud2, callback, queue_size=3)
    
    rospy.spin()