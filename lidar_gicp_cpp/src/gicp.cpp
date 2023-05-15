#include "lidar_gicp_cpp/gicp.hpp"

#include <iostream>
#include <chrono>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
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

GICP::GICP(ros::NodeHandle nh, ros::NodeHandle private_nh) : _prev_accX(0.0), _curr_accX(0.0), _prev_accY(0.0),
                                                             _curr_accY(0.0), _yaw_rate(0.0), _speedX(0.0),
                                                             _speedY(0.0), is_initial(true), is_imu_start(true)
{
    /*------------Parameters Definition-----------*/
    private_nh.param("leaf_size", _leaf_size, 0.1);
    ROS_INFO("leaf_size: %f", _leaf_size);

    private_nh.param("dist_threshold", _dist_threshold, 0.1);
    ROS_INFO("dist_threshold: %f", _dist_threshold, 0.1);

    private_nh.param("eps_angle", _eps_angle, 15);
    ROS_INFO("eps_angle: %d deg", _eps_angle);

    private_nh.param("mean_k", _mean_k, 50);
    ROS_INFO("mean_k: %d", _mean_k);

    private_nh.param("std_mul", _std_mul, 1.0);
    ROS_INFO("std_mul: %f", _std_mul);

    private_nh.param("transform_epsilon", _transform_epsilon, 0.001);
    ROS_INFO("transform_epsilon: %f", _transform_epsilon);

    private_nh.param("max_iterations", _max_iters, 1000);
    ROS_INFO("max_iterations: %d", _max_iters);

    private_nh.param("euclidean_fitness_epsilon", _euclidean_fitness_epsilon, 0.001);
    ROS_INFO("euclidean_fitness_epsilon: %f", _euclidean_fitness_epsilon);

    private_nh.param("max_correspondence_distance", _max_correspondence_distance, 1.0);
    ROS_INFO("max_correspondence_distance: %f", _max_correspondence_distance);

    private_nh.param<std::string>("point_cloud_topic", _point_cloud_topic, "velodyne_points");
    ROS_INFO("point_cloud_topic: %s", _point_cloud_topic.c_str());

    private_nh.param<std::string>("imu_topic", _imu_topic, "imu/data");
    ROS_INFO("imu_topic: %s", _imu_topic.c_str());

    private_nh.param<std::string>("path_topic", _path_topic, "path");
    ROS_INFO("path_topic: %s", _path_topic.c_str());

    /*-------------Pub-Sub Definition---------------*/
    pc_sub = nh.subscribe(_point_cloud_topic, 3, &GICP::cloudCallback, this);
    imu_sub = nh.subscribe(_imu_topic, 3, &GICP::imuCallback, this);
    path_pub = nh.advertise<nav_msgs::Path>("path", 3);
}

void GICP::removeNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sFilter;
    sFilter.setInputCloud(in_cloud_ptr);
    sFilter.setMeanK(_mean_k);
    sFilter.setStddevMulThresh(_std_mul);
    sFilter.filter(*out_cloud_ptr);
}

void GICP::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
    voxel.setInputCloud(in_cloud_ptr);
    voxel.filter(*out_cloud_ptr);
}

void GICP::removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane_ptr)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(_dist_threshold);
    seg.setAxis(Eigen::Vector3f(0, 0, 1)); // z- axis
    seg.setEpsAngle(_eps_angle);
    seg.setInputCloud(in_cloud_ptr);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
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

void GICP::filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_noise_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr only_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    downsampleCloud(in_cloud_ptr, voxel_cloud_ptr);
    removeNoise(voxel_cloud_ptr, no_noise_cloud_ptr);
    //removeGround(no_noise_cloud_ptr, no_ground_cloud_ptr, only_ground_cloud_ptr);
    *out_cloud_ptr = *no_noise_cloud_ptr;  
}

void GICP::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (is_imu_start)
    {
        _prev_accX = msg->linear_acceleration.x;
        _prev_accY = msg->linear_acceleration.y;
        _prev_imu_time = msg->header.stamp.toSec();

        is_imu_start = false;
    }

    else
    {
        _curr_accX = msg->linear_acceleration.x;
        _curr_accY = msg->linear_acceleration.y;
        _curr_imu_time = msg->header.stamp.toSec();

        double del_time = _curr_imu_time - _prev_imu_time;
        double avg_accX = (_prev_accX + _curr_accX) / 2;
        double avg_accY = (_prev_accY + _curr_accY) / 2;

        _speedX = avg_accX * del_time;
        _speedY = avg_accY * del_time;
        _yaw_rate = msg->angular_velocity.z;

        _prev_accX = _curr_accX;
        _prev_accY = _curr_accY;
        _prev_imu_time = _curr_imu_time;
    }
}

void GICP::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{   
    static int seq = 0;
    static nav_msgs::Path path;

    if(msg->data.size() == 0) return;
    
    if (is_initial)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg, *prev_cloud_ptr);
        filterCloud(prev_cloud_ptr, filtered_cloud_ptr);

        _prev_cloud = *filtered_cloud_ptr;
        _prev_time_stamp = msg->header.stamp.toSec();

        prev_transform = Eigen::Matrix4f::Identity();

        is_initial = false;
    }

    else
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(_prev_cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        pcl::fromROSMsg(*msg, *current_cloud_ptr);
        filterCloud(current_cloud_ptr, filtered_cloud_ptr);

        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setTransformationEpsilon(_transform_epsilon);
        gicp.setMaximumIterations(_max_iters);
        gicp.setMaxCorrespondenceDistance(_max_correspondence_distance);
        gicp.setEuclideanFitnessEpsilon(_euclidean_fitness_epsilon);
        gicp.setInputSource(filtered_cloud_ptr);
        gicp.setInputTarget(prev_cloud_ptr);

        double diff_time = msg->header.stamp.toSec() - _prev_time_stamp;
        double diff_yaw = diff_time * _yaw_rate;

        Eigen::AngleAxisf init_rotation(diff_yaw, Eigen::Vector3f::UnitZ());

        double del_x = diff_time * _speedX;
        double del_y = diff_time * _speedY;
        Eigen::Translation3f init_translation(del_x, del_y, 0.0);

        Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
        gicp.align(*transformed_cloud_ptr, init_guess);

        end = std::chrono::system_clock::now();

        std::cout << "GICP has converged: " << gicp.hasConverged() << "\tscore: " << gicp.getFitnessScore() << "\n";
        
        std::chrono::duration<double> elasped_seconds = end - start;
        std::cout << "elasped time: " << elasped_seconds.count() << "\n";

        Eigen::Matrix4f transform = gicp.getFinalTransformation();
        std::cout << "Final Transformation matrix\n" << transform << "\n";
        
        Eigen::Matrix4f curr_transform = prev_transform * transform;
        Eigen::Matrix3f rotation; // rotation matrix;
        Eigen::Vector3f translation; // translation matrix

        translation << curr_transform(0, 3), curr_transform(1, 3), curr_transform(2, 3);

        rotation << curr_transform(0, 0), curr_transform(0, 1), curr_transform(0, 2),
                    curr_transform(1, 0), curr_transform(1, 1), curr_transform(1, 2),
                    curr_transform(2, 0), curr_transform(2, 1), curr_transform(2, 2);

        Eigen::Quaternionf quat(rotation);


        path.header.seq = seq;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped curr_pose;
        curr_pose.header.seq = seq;
        curr_pose.header.frame_id = "map";
        curr_pose.header.stamp = ros::Time::now();

        curr_pose.pose.position.x = translation[0];
        curr_pose.pose.position.y = translation[1];
        curr_pose.pose.position.z = translation[2];
        curr_pose.pose.orientation.x = quat.x();
        curr_pose.pose.orientation.y = quat.y();
        curr_pose.pose.orientation.z = quat.z();
        curr_pose.pose.orientation.w = quat.w();

        path.poses.push_back(curr_pose);

        path_pub.publish(path);

        _prev_cloud = *filtered_cloud_ptr;
        prev_transform = curr_transform;
        _prev_time_stamp = msg->header.stamp.toSec();
        seq++;
    }
}