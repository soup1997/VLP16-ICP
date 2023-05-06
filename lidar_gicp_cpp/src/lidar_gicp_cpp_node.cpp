#include <ros/ros.h>
#include "lidar_gicp_cpp/gicp.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "gicp_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // using private parameters in node
    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}