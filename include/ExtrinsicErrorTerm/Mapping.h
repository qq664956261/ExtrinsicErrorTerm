#ifndef MAPPING_H
#define MAPPING_H
#define BOOST_TYPEOF_EMULATION
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <iostream>
#include <vector>
#include <utility>
#include <memory>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <omp.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/ceres.h"
#include "ceres/autodiff_cost_function.h"
#include "ExtrinsicErrorTerm/lidarFactor.h"
#include "ExtrinsicErrorTerm/aloam_analytic_factor.h"
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
enum SonarIndex
{
    left_front = 0,
    left_back = 1,

};

class Mapping
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    Mapping();
    ~Mapping();
    int readPose(const std::string filepath);
    int readSonarWaveData(const std::string filepath);
    int buildMap();
    int timeStampSynchronization(double sonarWaveTimeStamp);
    int Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    int Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Matrix3d R12, Eigen::Vector3d t12);
    void buildMultiFrame();
    double calculateDistance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2);
    void clusterPoses(double maxDistance);
    void multiFrameCombined();
    void map();
    void LoopClosure(const int index1, const int index2, const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2);
    int DetectLoopClosure(const Eigen::Matrix4d &pose, double &time_stamp);

    typedef std::shared_ptr<Mapping> Ptr;

protected:
    std::vector<std::vector<double>> _Poses;
    std::vector<std::vector<double>> _OptimizedPoses;
    std::vector<int> _optimizedPoseIndex;
    std::vector<std::vector<double>> _SonarWaveDatas;
    std::vector<std::vector<double>> _SonarWaveOptimizedDatas;
    std::deque<std::pair<int, double>> _PoseTimeStamp;
    std::deque<std::pair<int, double>> _SonarWaveTimeStamp;
    double _leftFrontFovRad{15};
    double _leftFrontX{0.156}; // sonar1坐标系相对base_link坐标系的x外参
    double _leftFronty{0.148}; // sonar1坐标系相对base_link坐标系的y外参
    // double _leftFrontX{0.136};  // sonar1坐标系相对base_link坐标系的x外参
    // double _leftFronty{0.128};  // sonar1坐标系相对base_link坐标系的y外参
    double _leftFrontyaw{1.57}; // sonar1坐标系相对base_link坐标系的yaw外参
    double _leftBackFovRad{15};
    double _leftBackX{-0.161};  // sonar2坐标系相对base_link坐标系的x外参
    double _leftBackY{0.182};   // sonar2坐标系相对base_link坐标系的x外参
    double _leftBackYaw{1.701}; // sonar2坐标系相对base_link坐标系的yaw外参
    pcl::PointCloud<pcl::PointXYZI>::Ptr _leftFrontCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _leftBackCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _optimizedCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudAll;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _linecloud;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr _kdtreeFromLeftBack;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr _kdtreeFromLeftFront;
    bool _leftBackBase{false};
    bool _useAutoDiff{true};
    bool _usePlaneConstraints{false};
    bool _useLineConstraints{false};
    double _firstLapEndTime;
    std::vector<std::pair<int, int>> _p_sonarindedx_poseindex;
    std::vector<std::pair<int, Eigen::Matrix4d>> _p_sonarindex_pose;
    std::vector<std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>>> _clustered_poses;
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>>> _p_cloud_pose;
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>>> _keyframes;
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>>> _keyframes_show;
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> _m_iNdt;
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> _m_icp;
    double _distance_threshold{3};
    bool _first_loop{false};
    int _loop_index{0};
};

#endif
