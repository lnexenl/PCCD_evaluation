//
// Created by lnex on 22-5-18.
//

#ifndef PCCDD_PCL_TYPES_HH
#define PCCDD_PCL_TYPES_HH
#include <filesystem>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
using namespace pcl;
using namespace octomap;
using namespace Eigen;
using string = std::string;
using ostream = std::ostream;

template<typename T>
using vector = std::vector<T>;


using PointXYZ = PointXYZ;
using PointCloudXYZ = PointCloud<PointXYZ>;
using PointXYZRGB = PointXYZRGB;
using PointCloudXYZRGB = PointCloud<PointXYZRGB>;

namespace fs = std::filesystem;

#endif //PCCDD_PCL_TYPES_HH
