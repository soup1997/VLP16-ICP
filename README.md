# VLP16-ICP

This is the repository using ICP(Iterative Closest Points) algorithm with velodyne VLP 16 pointcloud data.    
The rotational matrix $R$ and the translation matrix $t$ are obtained to estimate odometry.

- Input: `sensor_msgs/PointCloud2`, `sensor_msgs/Imu`
- Output: `nav_msgs/Path`

## Result
![image](https://github.com/soup1997/VLP16-ICP/assets/86957779/bb1ce2b6-d272-4eb5-b7b7-1f9b1c0dd851)

## Dataset
I used [Stevens-VLP16-Dataset](https://github.com/TixiaoShan/Stevens-VLP16-Dataset). you can download the dataset from the link.

## Usage
My environment is Ubuntu 20.04, ROS Noetic.

1) Place your downloaded bag file in any folder. Change the directory path in `launch/icp.launch`
```xml
<!-- Bag file play-->
<node pkg="rosbag" type="play" name="rosbag" args="your_bag_file_path"/>
```
2) just launch icp.launch
```bash
roslaunch lidar_icp_python icp.launch
roslaunch lidar_icp_cpp icp.launch
```

## Reference
* https://github.com/krishnasandeep09/ICP_Matching
* https://github.com/93won/2D_LiDAR_Odom_ICP
