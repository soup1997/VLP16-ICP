#include "gicp.h"

void colorize(const pcl::PointCloud<pcl::PointXYZ> & pc, pcl::PointCloud<pcl::PointXYZRGB> &pc_colored, const std::vector<int> & color) {
    int N = pc.points.size();
    pc_colored.clear();

    pcl::PointXYZRGB pt_tmp;
    for(auto i = 0; i < N; i++) {
        const auto &pt = pc.points[i];
        pt_tmp.x = pt.x; 
        pt_tmp.y = pt.y; 
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];

        pc_colored.points.emplace_back(pt_tmp);
    }
}