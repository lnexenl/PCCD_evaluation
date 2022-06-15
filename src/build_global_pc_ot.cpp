#include "change_evaluation.hpp"

int FX = 400, FY = 400;
int CX = 400, CY = 225;
double res = 2.4; //unused variable in this file

RadiusOutlierRemoval<PointXYZ> ror;
VoxelGrid<pcl::PointXYZ> vg;



int main(int argc, char** argv) {
    omp_set_num_threads(omp_get_thread_num());

    std::string workDIR = argv[1];
    std::string originPC = argv[2];
    std::string trial = argv[3];
    std::string tp = argv[4];
    int dist = atoi(argv[5]);

    std::map<uint64_t, pose> poseMap;
    std::map<uint64_t, cv::Mat> dispMap;

    fast_gicp::FastGICP<PointXYZ, PointXYZ> fgicp;

    PointCloudXYZ::Ptr priorOriginPC(new PointCloudXYZ()), priorChangedPC(new PointCloudXYZ()), globalPC(new PointCloudXYZ());
    PointCloudXYZ::Ptr newPC(new PointCloudXYZ()), rmPC(new PointCloudXYZ());
    PointCloudXYZ::Ptr priorNewPC(new PointCloudXYZ()), priorRmPC(new PointCloudXYZ());

    ror.setMinNeighborsInRadius(16);
    ror.setRadiusSearch(2.0);
    vg.setLeafSize(0.4, 0.4, 0.4);

    pcl::io::loadPCDFile(originPC, *priorOriginPC);
    pcl::io::loadPCDFile(workDIR + "/../new.pcd", *priorNewPC);
    pcl::io::loadPCDFile(workDIR + "/../removed.pcd", *priorRmPC);

    std::cout << "loading from: " << workDIR + "/" + trial + "/" << std::endl;

    loadPoseDisp(workDIR + "/" + trial + "/" + tp + "_global.txt",
                 workDIR + "/" + trial + "/disparity",
                 poseMap, dispMap);

    buildGLobalPointcloud(poseMap, dispMap, globalPC);
    // PointCloudXYZ::Ptr croppedPriorPC = filter_and_crop_target_cloud(globalPC, priorOriginPC);
    PointCloudXYZ::Ptr outputPC(new PointCloudXYZ);
    printf("Performing registration...\n");
    Eigen::Matrix4f m;
    align(fgicp, priorOriginPC, globalPC, m);
    pcl::transformPointCloud(*globalPC, *globalPC, m);
    pcl::io::savePCDFile(workDIR + "/" + trial + "/" + tp + "_" + std::to_string(dist) + "_global.pcd", *globalPC, true);
    octomap::OcTree obsArea = build_octomap_from_posedisp(m, poseMap, dispMap, dist);
    obsArea.writeBinary(workDIR + "/" + trial + "/" + tp + "_" + std::to_string(dist) + "_global.bt");

    return 0;
}
