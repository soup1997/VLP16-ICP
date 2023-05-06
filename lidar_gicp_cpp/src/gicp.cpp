#include "lidar_gicp_cpp/gicp.hpp"

#include <iostream>
#include <chrono>

#include <geometry_msgs/PoseWithCovariance.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/registration/gicp.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>


GICP::GICP(ros::NodeHandle nh, ros::NodeHandle private_nh): _prev_acc(0.0), _curr_acc(0.0), _yaw_rate(0.0), _speed(0.0), is_initial(true), is_imu_start(true) {
    private_nh.param("leaf_size", _leaf_size, 0.1);
    ROS_INFO("leaf_size: %f", _leaf_size);

    private_nh.param("dist_threshold", _dist_threshold, 0.1);
    ROS_INFO("dist_threshold: %f", _dist_threshold, 0.1);

    private_nh.param("eps_angle", _eps_angle, 15);
    ROS_INFO("eps_angle: %d deg", _eps_angle);

    private_nh.param("minX", _minX, 0.0);
    ROS_INFO("minX: %f", _minX);
    
    private_nh.param("minY", _minY, -25.0);
    ROS_INFO("minY: %f", _minY);
    
    private_nh.param("minZ", _minZ, -3.0);
    ROS_INFO("minZ: %f", _minZ);

    private_nh.param("maxX", _maxX, 40.0);
    ROS_INFO("maxX: %f", _maxX);
    
    private_nh.param("maxY", _minY, -25.0);
    ROS_INFO("maxY: %f", _maxY);
    
    private_nh.param("maxZ", _maxZ, -3.0);
    ROS_INFO("maxZ: %f", _maxZ);

    private_nh.param("mean_k", _mean_k, 50);
    ROS_INFO("mean_k: %d", _mean_k);
    
    private_nh.param("std_mul", _std_mul, 1.0);
    ROS_INFO("std_mul: %f", _std_mul);


    private_nh.param("transform_epsilon", _transform_epsilon, 0.01);
    ROS_INFO("transform_epsilon: %f", _transform_epsilon);
    
    private_nh.param("max_iterations", _max_iters, 75);
    ROS_INFO("max_iterations: %d", _max_iters);
    
    private_nh.param("euclidean_fitness_epsilon", _euclidean_fitness_epsilon, 0.1);
    ROS_INFO("euclidean_fitness_epsilon: %f", _euclidean_fitness_epsilon);
    
    private_nh.param("max_correspondence_distance", _max_correspondence_distance, 1.0);
    ROS_INFO("max_correspondence_distance: %f", _max_correspondence_distance);

    private_nh.param<std::string>("point_cloud_topic", _point_cloud_topic, "velodyne_points");
    ROS_INFO("point_cloud_topic: %s", _point_cloud_topic.c_str());
    
    private_nh.param<std::string>("imu_topic", _imu_topic, "imu/data");
    ROS_INFO("imu_topic: %s", _imu_topic.c_str());
    
    private_nh.param<std::string>("pose_topic", _pose_topic, "icp_pose");
    ROS_INFO("pose_topic: %s", _pose_topic.c_str());

    pc_sub = nh.subscribe(_point_cloud_topic, 1, &GICP::cloudCallback, this);
    imu_sub = nh.subscribe(_imu_topic, 1, &GICP::imuCallback, this);
    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(_pose_topic, 1);
}

void GICP::cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr) {
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setInputCloud(in_cloud_ptr);
    boxFilter.setMin(Eigen::Vector4f(_minX, _minY, _minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(_maxX, _maxY, _maxZ, 1.0));
    boxFilter.filter(*out_cloud_ptr);
}

void GICP::removeNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sFilter;
    sFilter.setInputCloud(in_cloud_ptr);
    sFilter.setMeanK(_mean_k);
    sFilter.setStddevMulThresh(_std_mul);
    sFilter.filter(*out_cloud_ptr);
}

void GICP::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr) {
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(in_cloud_ptr);
    voxel.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
    voxel.filter(*out_cloud_ptr);
}

void GICP::removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane_ptr) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setInputCloud(in_cloud_ptr);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(_dist_threshold);
    seg.setAxis(Eigen::Vector3f(0, 0, 1)); // z- axis
    seg.setEpsAngle(_eps_angle);
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size() == 0) {
        std::cout << "Could not estimate the plane" << std::endl;
    }

    // remove ground from the cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(in_cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true); // true removes the indices
    extract.filter(*out_cloud_ptr);

    extract.setNegative(false); // false leaves only the indices
    extract.filter(*ground_plane_ptr);
}

void GICP::filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr only_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr no_noise_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    cropCloud(in_cloud_ptr, cropped_cloud_ptr);
    downsampleCloud(cropped_cloud_ptr, downsampled_cloud_ptr);
    removeGround(downsampled_cloud_ptr, no_ground_cloud_ptr, only_ground_cloud_ptr);
}

void GICP::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (is_imu_start) {
        _prev_acc = msg->linear_acceleration.x;
        _prev_imu_time = msg->header.stamp.toSec();

        is_imu_start = false;
    }

    else {
        _curr_acc = msg->linear_acceleration.x;
        _curr_imu_time = msg->header.stamp.toSec();

        double del_time = _curr_imu_time - _prev_imu_time;
        double avg_acc = (_prev_acc + _curr_acc) / 2;

        _speed = avg_acc * del_time;
        _yaw_rate = msg->angular_velocity.z;

        _prev_acc = _curr_acc;
        _prev_imu_time = _curr_imu_time;
    }
}

void GICP::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    
}