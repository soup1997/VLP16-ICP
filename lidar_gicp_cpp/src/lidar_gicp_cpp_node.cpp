#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include "gicp.h"

void callback(const sensor_msgs::PointCloud2 &msg) {

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gicp_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("velodyne_points", 3, callback);
    ros::spin();
    
    return 0;
}