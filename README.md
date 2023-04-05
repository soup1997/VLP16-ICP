# VLP16-ICP

This is the repository for using the icp algorithm with 3d lidar data. the rotational matrix $R$ and the translation matrix $t$ are obtained to finally estimate odometry.

## Result
![image](https://user-images.githubusercontent.com/86957779/230062796-7d048fb0-785d-4389-b153-3f98743d8684.png)

## Dataset
I used [Stevens-VLP16-Dataset](https://github.com/TixiaoShan/Stevens-VLP16-Dataset). you can download the dataset from the link.

## Usage
```bash
asdasd
```

## To do
* Transform local coordinate to global coordinate
* sensor fusion with imu(to get rotation matrix)
* roscpp implementation

## Reference
* http://daddynkidsmakers.blogspot.com/2021/09/icpiterative-closest-point.html
* https://github.com/93won/2D_LiDAR_Odom_ICP
