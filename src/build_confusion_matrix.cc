#include "change_evaluation.hpp"
#include <pcl/filters/crop_box.h>
int FX = 400, FY = 400;
int CX = 400, CY = 225;
double res;

RadiusOutlierRemoval<PointXYZ> ror;
VoxelGrid<pcl::PointXYZ> vg;
 

int main(int argc, char **argv) {
    std::string workDIR = argv[1];
    std::string trial = argv[2];
    std::string tp = argv[3];
    int dist = atoi(argv[4]);
    res = atof(argv[5]);
    PointCloudXYZ::Ptr priorOriginPC(new PointCloudXYZ()), priorChangedPC(new PointCloudXYZ()), globalPC(new PointCloudXYZ());
    PointCloudXYZ::Ptr newPC(new PointCloudXYZ()), rmPC(new PointCloudXYZ());
    PointCloudXYZ::Ptr priorNewPC(new PointCloudXYZ()), priorRmPC(new PointCloudXYZ());
    CropBox<PointXYZ> cb;
    cb.setMax({ 125., 125., 50., 1. });
    cb.setMin({ -125., -125., -10., 1. });
    pcl::io::loadPCDFile("/home/lnex/dataset/origin/4x.pcd", *priorOriginPC);
    pcl::io::loadPCDFile(workDIR + "/../new.pcd", *priorNewPC);
    pcl::io::loadPCDFile(workDIR + "/../removed.pcd", *priorRmPC);
    pcl::io::loadPCDFile(workDIR + "/" + trial + "/" + tp + "_" + std::to_string(dist) + "_global.pcd", *globalPC);
    cb.setInputCloud(globalPC);
    cb.filter(*globalPC);
    ror.setMinNeighborsInRadius(16);
    ror.setRadiusSearch(2.0);
    vg.setLeafSize(0.4, 0.4, 0.4);
    octomap::OcTree obsArea(workDIR + "/" + trial + "/" + tp + "_" + std::to_string(dist) + "_global.bt");

    PointCloudXYZ::Ptr intersectPC = build_intersect_prior_pointcloud(obsArea, priorOriginPC, globalPC);

    build_diff_clouds(intersectPC, globalPC, newPC, rmPC);
    pcl::io::savePCDFile(workDIR + "/" + trial + "/" + tp + "_" + std::to_string(dist) + "_intersect.pcd", *intersectPC, true);
    pcl::io::savePCDFile(workDIR + "/" + trial + "/" + tp + "_" + std::to_string(dist) + "_new.pcd", *newPC, true);
    pcl::io::savePCDFile(workDIR + "/" + trial + "/" + tp + "_" + std::to_string(dist) + "_rm.pcd", *rmPC, true);
    vector<double> metrics = calculate_metrics(obsArea, globalPC, priorNewPC, priorRmPC, newPC, rmPC);
    std::cout << metrics[0] << "," << metrics[1] << "," << metrics[2] << "," << metrics[3] << "," << metrics[4] << "," << metrics[5] << std::endl;
}
