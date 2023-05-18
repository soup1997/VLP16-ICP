#include <ros/ros.h>
#include "lidar_icp_cpp/icp.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gicp_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // using private parameters in node
    ros::Rate loop_rate(10);

    ICP icp(nh, private_nh);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}