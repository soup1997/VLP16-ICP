#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from utils.processing import *


def callback(msg):
    global publisher
    
    while msg.data is None:
        rospy.loginfo('Lidar data is empty...')
    
    pcd = pre_processing(msg)
    publisher.publish(pcd)    
        
if __name__ == '__main__':
    rospy.init_node('ICP', anonymous=True)
    rospy.loginfo('ICP' + ' is now working...')
    publisher = rospy.Publisher('/pre_pcd', PointCloud2, queue_size=10)
    rospy.Subscriber('/velodyne_points', PointCloud2, callback, queue_size=10)
    
    rospy.spin()