#ifndef __GICP_H__
#define __GICP_H__

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <chrono>

void colorize(const pcl::PointCloud<pcl::PointXYZ> & pc, pcl::PointCloud<pcl::PointXYZRGB> &pc_colored, const std::vector<int> & color);

#endif