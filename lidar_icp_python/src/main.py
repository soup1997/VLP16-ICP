#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
from utils.utils import *
from utils.icp import icp

prev_pose = [0, 0, 0, 0, 0, 0] # 6 DOF(x, y, z, roll, pitch, yaw)
prev_pcd = None


def callback(pcd):
    global prev_pose, prev_pcd

    current_pcd = pcd_processing(pcd)  # (18000, 3) shape

    if prev_pcd is not None:
        H = icp(A=current_pcd, B=prev_pcd)

    else:
        H = icp(A=current_pcd, B=current_pcd)

    curr_pose = get_pose(transform=H, pose=prev_pose)
    draw_pose(curr_pose)

    prev_pose = curr_pose
    prev_pcd = current_pcd


if __name__ == '__main__':
    rospy.init_node('ICP', anonymous=True)
    rospy.loginfo('ICP' + ' is now working...')
    rospy.Subscriber('/velodyne_points', PointCloud2, callback, queue_size=3)

    while not rospy.is_shutdown():
        rospy.spin()
