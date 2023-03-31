#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2
from open3d_ros_helper import open3d_ros_helper as orh

def convert_type1(pcd):
    return orh.rospc_to_o3dpc(pcd, remove_nans=True)

def voxel_down(pcd):
    return pcd.voxel_down_sample(voxel_size=0.02)

def outlier_removal(pcd):
    pcd = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return pcd[0]

def convert_type2(pcd):
    return orh.o3dpc_to_rospc(pcd, frame_id='velodyne')    

def pre_processing(pcd):
    pcd = convert_type1(pcd)
    pcd = voxel_down(pcd)
    pcd = outlier_removal(pcd)
    pcd = convert_type2(pcd)
    
    return pcd