#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array
from open3d_ros_helper import open3d_ros_helper as orh
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler

path = Path()
path_pub = rospy.Publisher('path', Path, queue_size=3)
seq = 0

def v2t(pose):
    '''
    Input: pose vector, includes x, y, heading information
    
    Output: return Homogeneous matrix, which has 3 X 3 shape
    '''
    # from vector to transfrom
    tx = pose[0]
    ty = pose[1]
    theta = pose[2]

    transform = np.array([[np.cos(theta), -np.sin(theta), tx],
                          [np.sin(theta), np.cos(theta), ty],
                          [0, 0, 1]], dtype=np.float32)

    return transform


def t2v(transform):
    '''
    Input: return Homogeneous matrix, which has 3 X 3 shape
    Output: pose vector, includes [x, y, heading], (3, ) shape
    '''
    v = np.zeros((3,))
    v[:2] = transform[:2, 2]  # tx, ty
    # atan(np.sin(theta), np.cos(theta))
    v[2] = np.arctan2(transform[1, 0], transform[0, 0])

    return v


def draw_pose(pose):
    global seq, path
    print("Current Pose: {}".format(pose))
    
    path.header.seq = seq
    path.header.frame_id = 'map'
    path.header.stamp = rospy.Time.now()
    
    pose2d = PoseStamped()
    pose2d.pose.position.x = pose[0]
    pose2d.pose.position.y = pose[1]
    pose2d.pose.position.z = 0
    
    q = quaternion_from_euler(0, 0, pose[2])
    pose2d.pose.orientation.x = q[0]
    pose2d.pose.orientation.y = q[1]
    pose2d.pose.orientation.z = q[2]
    pose2d.pose.orientation.w = q[3]
    
    path.poses.append(pose2d)
    
    path_pub.publish(path)
    seq += 1


# ================= Lidar Pre-processing Function =================#
def cvt2_open3d(pcd):
    return orh.rospc_to_o3dpc(pcd, remove_nans=True)


def voxel_down(pcd):
    return pcd.voxel_down_sample(voxel_size=0.05)


def outlier_removal(pcd):
    pcd = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return pcd[0]


def cvt2_pcd(pcd):
    return orh.o3dpc_to_rospc(pcd, frame_id='velodyne')


def pre_processing(pcd):
    '''
    Execute Lidar pre-processing pipeline\  
    Input:
        convert from ros pointcloud2 to open3d pcd
    Output:
        convert from open3d pcd to ros pointcloud2
    '''
    pcd = cvt2_open3d(pcd)
    pcd = voxel_down(pcd)
    pcd = outlier_removal(pcd)
    pcd = cvt2_pcd(pcd)
    pcd = pointcloud2_to_xyz_array(pcd, remove_nans=True)
    return pcd