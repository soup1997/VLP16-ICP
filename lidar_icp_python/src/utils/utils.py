#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array
from scipy.spatial.transform import Rotation
from open3d_ros_helper import open3d_ros_helper as orh
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import *

path = Path()
path_pub = rospy.Publisher('path', Path, queue_size=3)
seq = 0


def local2global(pose, pcd):
    x, y, z, roll, pitch, yaw = pose
    rotation_mat = euler_matrix(roll, pitch, yaw)
    transform_mat = np.eye(4)
    transform_mat[:3, :3] = rotation_mat
    transform_mat[:3, 3] = [x, y, z]
    
    global_pcd = np.hstack((pcd, np.ones((pcd.shape[0], 1))))
    global_pcd = np.dot(transform_mat, global_pcd.T).T[:, :3]
    
    return global_pcd
    

def get_pose(transform, pose):
    q_rot = quaternion_from_matrix(transform)
    q_origin = quaternion_from_euler(pose[3], pose[4], pose[5])

    translation = transform[:3, 3]
    tx, ty, tz = translation[0], translation[1], translation[2]

    q_new = quaternion_multiply(q_rot, q_origin)
    rpy_new = euler_from_quaternion(q_new)

    pose[0] += tx
    pose[1] += ty
    pose[2] += tz

    pose[3] = rpy_new[0]
    pose[4] = rpy_new[1]
    pose[5] = rpy_new[2]

    return pose


def draw_pose(pose):
    global seq, path

    path.header.seq = seq
    path.header.frame_id = 'map'
    path.header.stamp = rospy.Time.now()

    pose2d = PoseStamped()
    pose2d.pose.position.x = pose[0]
    pose2d.pose.position.y = pose[1]
    pose2d.pose.position.z = pose[2]

    q = quaternion_from_euler(pose[3], pose[4], pose[5])
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
    pcd = pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=10.0)
    return pcd[0]


def cvt2_pcd(pcd):
    return orh.o3dpc_to_rospc(pcd, frame_id='velodyne')


def pcd_processing(pcd):
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
