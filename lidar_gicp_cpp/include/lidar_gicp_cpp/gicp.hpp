#ifndef LIDAR_GICP_CPP_GICP_HPP
#define LIDAR_GICP_CPP_GICP_HPP

#include <ros/ros.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <string>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

class GICP
{
public:
    GICP(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~GICP(){};

private:
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void removeNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr);
    void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr);
    void removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane_ptr);
    void filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr);

    ros::Subscriber pc_sub;
    ros::Subscriber imu_sub;
    ros::Publisher path_pub;

    pcl::PointCloud<pcl::PointXYZ> _prev_cloud;
    Eigen::Matrix4f prev_transform;
    bool is_initial, is_imu_start;
    double _prev_accX, _curr_accX;
    double _prev_accY, _curr_accY;

    double _prev_imu_time, _curr_imu_time;
    double _prev_time_stamp;

    /*---------sub-pub parameters---------*/
    std::string _point_cloud_topic;
    std::string _imu_topic;
    std::string _path_topic;

    /*---------GICP parameters---------*/
    double _leaf_size;                               // leaf size for voxel grid
    int _mean_k;                                     // number of neighbors to analyze for each point for noise removal
    double _std_mul;                                 // standard deviation multiplication threshold for noise removal

    double _dist_threshold; // distance threshold for RANSAC to consider a point as linear
    int _eps_angle;         // allowed difference of angles in degrees for perpendicular plane model

    double _transform_epsilon;           // minimum transformation difference for termination condition
    int _max_iters;                      // max number of registration iterations
    double _euclidean_fitness_epsilon;   // maximum allowd Euclidean error between two consecutive steps in the ICP loop
    double _max_correspondence_distance; // correspondences with higher distandces will be ignored
    double _speedX;                      // speedX for initial guess
    double _speedY;                      // speedY for initial guess
    double _yaw_rate;                    // change in yaw for initial guess
};
#endif