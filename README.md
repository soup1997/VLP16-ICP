# VLP16-ICP

This is the repository for using the icp algorithm with 3d lidar data.    
The rotational matrix $R$ and the translation matrix $t$ are obtained to finally estimate odometry.

## Result
![image](https://user-images.githubusercontent.com/86957779/230062796-7d048fb0-785d-4389-b153-3f98743d8684.png)

## Dataset
I used [Stevens-VLP16-Dataset](https://github.com/TixiaoShan/Stevens-VLP16-Dataset). you can download the dataset from the link.

## Usage
My environment is Ubuntu 20.04, ROS Noetic.
1) Run the following command
```bash
pip install -r requirements.txt
```
2) Place your downloaded bag file in any folder. Change the directory path in `launch/icp.launch`
```xml
<!-- Bag file play-->
<node pkg="rosbag" type="play" name="rosbag" args="-l /downloaded_dataset/test.bag"/>
```
3) just launch icp.launch
```bash
roslaunch lidar_icp_python icp.launch
```

## To do
- [ ] Transform local coordinate to global coordinate
- [ ] usage of imu
- [ ] roscpp implementation

## Reference
* https://github.com/krishnasandeep09/ICP_Matching
* https://github.com/93won/2D_LiDAR_Odom_ICP
