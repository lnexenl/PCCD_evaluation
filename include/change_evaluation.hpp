/*******************************************************
* Copyright (C) 2022
* Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
* Department of Electronic Engineering, Tsinghua University.
* Created by Zihan Lin on 2022/5/28.
*******************************************************/
#include <chrono>
#include <cstdio>
#include <fstream>
#include <map>
#include <ostream>
#include <string>

#include <Eigen/Dense>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/filesystem.hpp>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "alias.h"
extern RadiusOutlierRemoval<PointXYZ> ror;
extern VoxelGrid<PointXYZ> vg;
extern int CX, CY, FX, FY;
extern double res;

class pose
{
public:
    uint64_t stamp;
    double x, y, z, qx, qy, qz, qw;
    friend ostream& operator<<(ostream& out, pose p) {
        return out << p.stamp << " " << p.x << " " << p.y << " " << p.z << " " << p.qx << " " << p.qy << " " << p.qz << " " << p.qw << std::endl;
    }
};

using dispmap = std::map<uint64_t, cv::Mat>;
using posemap = std::map<uint64_t, pose>;

void loadPoseDisp(string poseFile, string dispDir, posemap& poseMap, dispmap& dispMap) {
    pose p;
    std::fstream paths(poseFile, std::ios::in);
    while (paths >> p.stamp >> p.x >> p.y >> p.z >> p.qx >> p.qy >> p.qz >> p.qw) {
        poseMap[uint64_t(p.stamp)] = p;
    }
    paths.close();

    for (auto const& dir_entry : fs::directory_iterator(dispDir)) {
        if (boost::algorithm::ends_with(dir_entry.path().string(), ".png")) {
            vector<string> tmp;
            cv::Mat img;
            boost::algorithm::split(tmp, dir_entry.path().string(), boost::is_any_of("/"));
            boost::algorithm::split(tmp, tmp.back(), boost::is_any_of("."));
            img = cv::imread(dir_entry.path().string(), cv::IMREAD_UNCHANGED);
            img.convertTo(img, CV_16UC1);
            //            pcl::io::loadPCDFile(dir_entry.path().string(), tmppc);
            dispMap[uint64_t(std::stoull(tmp[0]))] = img;
        }
    }
    std::cout << "load " << dispMap.size() << " poses and " << dispMap.size() << " disparities." << std::endl;
}

void buildGLobalPointcloud(std::map<uint64_t, pose>& poseMap, std::map<uint64_t, cv::Mat>& dispMap, PointCloudXYZ::Ptr globalpc) {
    int cnt = 0;
    for (const auto& it : dispMap) {
        auto start = std::chrono::system_clock::now();
        PointCloudXYZ::Ptr pts(new PointCloudXYZ());
        if (poseMap.find(it.first) != poseMap.end()) {
            cv::Mat disp = it.second;
            uint64_t stamp = it.first;
            pose p = poseMap[stamp];
            disp.convertTo(disp, CV_32FC1, 1. / 256.);
            //            std::cout<<disp<<std::endl;
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            PointCloudXYZ tmppc;
            T.block<3, 3>(0, 0) = Eigen::Quaternionf(poseMap[stamp].qw, poseMap[stamp].qx, poseMap[stamp].qy, poseMap[stamp].qz).matrix();
            T.block<3, 1>(0, 3) << poseMap[stamp].x, poseMap[stamp].y, poseMap[stamp].z;
            //            std::cout << T << std::endl;
            PointCloudXYZ::Ptr tmpptr(new PointCloudXYZ);
            int d = 0;

            for (int i = 0; i < disp.rows; ++i)
                for (int j = 0; j < disp.cols; ++j) {
                    if (disp.at<float>(i, j) < 8) {
                        continue;
                    }
                    else {
                        Eigen::Vector3f pt = { (float(j) - float(CX)) / float(FX), (float(i) - float(CY)) / float(FY), 1.0 };
                        d = 400. / disp.at<float>(i, j);
                        pt = d * pt;
                        pt = Eigen::Vector3f{ pt.z() + 2.0, pt.x() - 0.5, pt.y() - 2.5 };
                        pt = T.block<3, 3>(0, 0) * pt + T.block<3, 1>(0, 3);
                        pts->push_back({ pt.x(), pt.y(), pt.z() });
                    }
                }
        }
        vg.setInputCloud(pts);
        vg.filter(*pts);
        ror.setInputCloud(pts);
        ror.filter(*pts);
        *globalpc += *pts;
        cnt++;
    }
    vg.setInputCloud(globalpc);
    vg.filter(*globalpc);
}

// template <typename Registration>
void align(pcl::Registration<PointXYZ, PointXYZ>& reg, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source, Eigen::Matrix4f &transform) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);

    double fitness_score = 0.0;

    // single run
    auto t1 = std::chrono::high_resolution_clock::now();
    // fast_gicp reuses calculated covariances if an input cloud is the same as the previous one
    // to prevent this for benchmarking, force clear source and target clouds
    reg.setInputTarget(target);
    reg.setInputSource(source);
    reg.align(*aligned);
    auto t2 = std::chrono::high_resolution_clock::now();
    fitness_score = reg.getFitnessScore();
    double single = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;

    std::cout << "single:" << single << "[msec] " << std::endl;
    transform = reg.getFinalTransformation();
    Eigen::Quaternionf q(transform.block<3, 3>(0, 0));
    printf("%1.4f %1.4f %1.4f %1.4f    %2.4f %2.4f %2.4f\n", q.w(), q.x(), q.y(), q.z(), transform.coeff(0, 3), transform.coeff(1, 3), transform.coeff(2, 3));
    if (reg.hasConverged()) {
        printf("fgicp converged, fitness:%f\n", reg.getFitnessScore());
        Eigen::Matrix4f m = reg.getFinalTransformation();
    }
}

PointCloudXYZ::Ptr filter_and_crop_target_cloud(const PointCloudXYZ::Ptr& globalPC, const PointCloudXYZ::Ptr& priorPC) {
    printf("performing filter and crop\n");
    PointCloudXYZ::Ptr cropped_priorPC(new PointCloudXYZ());
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*globalPC, min_pt, max_pt);
    pcl::BoxClipper3D<pcl::PointXYZ> boxclip(Eigen::Affine3f::Identity());
    Eigen::Vector3f sca(1. / ((max_pt.x - min_pt.x) / 2 + 0.5),
                        1. / ((max_pt.y - min_pt.y) / 2 + 0.5),
                        1. / ((max_pt.z - min_pt.z) / 2 + 0.5));
    Eigen::Vector3f tr((max_pt.x + min_pt.x) / 2, (max_pt.y + min_pt.y) / 2, (max_pt.z + min_pt.z) / 2);
    Eigen::Affine3f t(Eigen::Translation3f(-tr.cwiseProduct(sca)));
    t.scale(sca);
    boxclip.setTransformation(t);
    pcl::IndicesPtr ind(new pcl::Indices());
    boxclip.clipPointCloud3D(*priorPC, *ind);
    pcl::ExtractIndices<pcl::PointXYZ> extractind;
    extractind.setInputCloud(priorPC);
    extractind.setIndices(ind);
    extractind.filter(*cropped_priorPC);
    return cropped_priorPC;
}

OcTree build_octomap_from_posedisp(const Eigen::Matrix4f& m, posemap& poseMap, dispmap& dispMap, int free_dist) {
    omp_set_num_threads(omp_get_max_threads());
    octomap::OcTree tree(0.8);
    int cnt = 0;
    // auto start = std::chrono::system_clock::now();
    printf("%zu %zu\n", dispMap.begin()->first, poseMap.begin()->first);
    VoxelGrid<PointXYZ> vgg;
    vgg.setLeafSize(0.8, 0.8, 0.8);
    for (const auto& it : dispMap) {
        cnt++;
        if (cnt % 2)
            continue;

        if (poseMap.find(it.first) != poseMap.end()) {
            cv::Mat disp = it.second;
            uint64_t stamp = it.first;
            disp.convertTo(disp, CV_32FC1, 1. / 256.);
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            PointCloudXYZ tmppc;
            
            T.block<3, 3>(0, 0) = Eigen::Quaternionf(poseMap[stamp].qw, poseMap[stamp].qx, poseMap[stamp].qy, poseMap[stamp].qz).matrix();
            T.block<3, 1>(0, 3) << poseMap[stamp].x, poseMap[stamp].y, poseMap[stamp].z;

            PointCloudXYZ::Ptr tmpptr(new PointCloudXYZ);
            int d = 0;
            octomap::Pointcloud empty_pts, pts;
            PointCloudXYZ::Ptr empty_cloud(new PointCloudXYZ()), cloud(new PointCloudXYZ());
            for (int i = 0; i < disp.rows; ++i)
                for (int j = 0; j < disp.cols; ++j) {
                    Eigen::Vector3f pt = { (float(j) - float(CX)) / float(FX), (float(i) - float(CY)) / float(FY), 1.0 };
                    if (disp.at<float>(i, j) < 2) {
                        // printf("1\n");
                        if (i > 450 / 2)
                            continue;
                        if ( j < 100 || j > 750)
                            continue;
                        pt = 100 * pt;
                        pt = Eigen::Vector3f{ pt.z() + 2.0, pt.x() - 0.5, pt.y() - 2.5 };
                        pt = T.block<3, 3>(0, 0) * pt + T.block<3, 1>(0, 3);
                        empty_cloud->push_back({ pt.x(), pt.y(), pt.z() });
                    }
                    else {
                        d = 400. / disp.at<float>(i, j);
                        if (d > 50)
                            continue;
                        else {
                            pt = d * pt;
                            pt = Eigen::Vector3f{ pt.z() + 2.0, pt.x() - 0.5, pt.y() - 2.5 };
                            pt = T.block<3, 3>(0, 0) * pt + T.block<3, 1>(0, 3);
                            cloud->push_back({ pt.x(), pt.y(), pt.z() });
                        }
                    }
                }
            Eigen::Vector3f cam_p(poseMap[stamp].x, poseMap[stamp].y, poseMap[stamp].z);
            cam_p = m.block<3, 3>(0, 0) * cam_p + m.block<3, 1>(0, 3);
            vg.setLeafSize(0.8, 0.8, 0.8);
            vg.setInputCloud(cloud);
            vg.filter(*cloud);
            ror.setInputCloud(cloud);
            ror.filter(*cloud);
            vgg.setInputCloud(cloud);
            vgg.filter(*cloud);
            vgg.setInputCloud(empty_cloud);
            vgg.filter(*empty_cloud);
            pcl::transformPointCloud(*cloud, *cloud, m);
            pcl::transformPointCloud(*empty_cloud, *empty_cloud, m);
            for (auto pt : cloud->points)
                pts.push_back({ pt.x, pt.y, pt.z });
            for (auto pt : empty_cloud->points)
                empty_pts.push_back({ pt.x, pt.y, pt.z });
            
            
            // std::cout << cam_p << std::endl;
            tree.insertPointCloud(empty_pts, octomap::point3d(cam_p[0], cam_p[1], cam_p[2]), free_dist, true);
            tree.insertPointCloud(pts, octomap::point3d(cam_p[0],cam_p[1],cam_p[2]), -1, true);
        }
    }
    tree.updateInnerOccupancy();
    return tree;
}

PointCloudXYZ::Ptr build_intersect_prior_pointcloud(const OcTree& obsArea, const PointCloudXYZ::Ptr priorOriginPC, const PointCloudXYZ::Ptr globalPC) {
    PointCloudXYZ::Ptr intersectPC(new PointCloudXYZ());
    pcl::search::KdTree<PointXYZ> kdt, kdt1;
    kdt.setEpsilon(0.001);
    kdt1.setEpsilon(0.001);
    kdt.setInputCloud(globalPC);
    PointCloudXYZ::Ptr obsPC(new PointCloudXYZ());
    vector<float> tv;
    Indices ind;
    PointCloudXYZ::Ptr pc(new PointCloudXYZ());

    // for (OcTree::leaf_iterator it = obsArea.begin_leafs(), end = obsArea.end_leafs(); it != end; ++it)
    // {
    //     PointXYZ pt;
    //     octomap::point3d p;
    //     p = it.getCoordinate();
    //     pt.x = p.x();
    //     pt.y = p.y();
    //     pt.z = p.z();
    //     pc->push_back(pt);
    // }
    // kdt1.setInputCloud(pc)
    for (auto pt : priorOriginPC->points) {
        if (obsArea.search(pt.x, pt.y, pt.z, obsArea.getTreeDepth()) != NULL || kdt.radiusSearch(pt, res, ind, tv) > 0)
            intersectPC->emplace_back(pt);
    }
    return intersectPC;
}

void build_diff_clouds(const PointCloudXYZ::Ptr& intersectPC, const PointCloudXYZ::Ptr& globalPC, PointCloudXYZ::Ptr& newPC, PointCloudXYZ::Ptr& rmPC) {
    // printf("building new and removed pointclouds.\n");
    pcl::search::KdTree<PointXYZ> kdt;
    kdt.setEpsilon(0.001);
    kdt.setInputCloud(intersectPC);

    Indices ind;
    vector<float> dist;

    for (auto pt : globalPC->points) {
        if (kdt.radiusSearch(pt, res, ind, dist) == 0)
            newPC->emplace_back(pt);
    }

    kdt.setInputCloud(globalPC);

    for (auto pt : intersectPC->points) {
        if (kdt.radiusSearch(pt, res, ind, dist) == 0)
            rmPC->emplace_back(pt);
    }

    ror.setInputCloud(newPC);
    ror.filter(*newPC);
    ror.setInputCloud(rmPC);
    ror.filter(*rmPC);

    // printf("new points: %lu, removed points: %lu\n", newPC->size(), rmPC->size());
}

vector<double> calculate_metrics(OcTree obsArea, const PointCloudXYZ::Ptr &globalPC,
                       const PointCloudXYZ::Ptr &priorNewPC, const PointCloudXYZ::Ptr &priorRmPC, 
                       const PointCloudXYZ::Ptr &newPC, const PointCloudXYZ::Ptr &rmPC) {
    PointCloudXYZ::Ptr priorObsNewPC(new PointCloudXYZ()), priorObsRmPC(new PointCloudXYZ());
    pcl::search::KdTree<PointXYZ> kdt;
    kdt.setEpsilon(0.01);
    Indices ind;
    vector<float> dist;

    kdt.setInputCloud(globalPC);
    for (auto pt : priorNewPC->points) {
        if (obsArea.search(pt.x, pt.y, pt.z, 0) != NULL || kdt.radiusSearch(pt, res, ind, dist) > 0)
            priorObsNewPC->emplace_back(pt);
    }
    for (auto pt:priorRmPC->points) {
        if (obsArea.search(pt.x, pt.y, pt.z, 0) != NULL)
            priorObsRmPC->emplace_back(pt);
    }

    int priorNN = 0, globalNN = 0, priorRR = 0, globalRR = 0;

    if (priorObsNewPC->size() && newPC->size()) {
        kdt.setInputCloud(priorObsNewPC);
        for (auto pt:newPC->points) {
            if (kdt.radiusSearch(pt, res, ind, dist) > 0)
                globalNN++;
        }
        kdt.setInputCloud(newPC);
        for (auto pt:priorObsNewPC->points) {
            if (kdt.radiusSearch(pt, res, ind, dist) > 0)
                priorNN++;
        }
    }

    if (priorObsRmPC->size() && rmPC->size()) {
        kdt.setInputCloud(priorObsRmPC);
        for (auto pt:rmPC->points) {
            if (kdt.radiusSearch(pt, res, ind, dist) > 0)
                globalRR++;
        }
        kdt.setInputCloud(rmPC);
        for (auto pt:priorObsRmPC->points) {
            if (kdt.radiusSearch(pt, res, ind, dist) > 0)
                priorRR++;
        }
    }
    // recall, precision of new, recall, precision of removed.
    vector<double> result;
    result.push_back(100. * double(priorNN) / double(priorObsNewPC->size()));
    result.push_back(100. * double(globalNN) / double(newPC->size()));
    result.push_back(100. * double(priorRR) / double(priorObsRmPC->size()));
    result.push_back(100. * double(globalRR) / double(rmPC->size()));
    result.push_back(priorNN);
    result.push_back(priorRR);
    return result;
}
