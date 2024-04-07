#include <ExtrinsicErrorTerm/Mapping.h>
Mapping::Mapping()
{
    _leftFrontCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    _leftBackCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    _CloudAll.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    _kdtreeFromLeftBack.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    _kdtreeFromLeftFront.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
}
Mapping::~Mapping()
{
}

int Mapping::readPose(const std::string filepath)
{
    int ret = -1;
    std::string filenames = filepath + "/ArmyOdom.txt";
    std::ifstream infile(filenames.c_str());
    int count = 0;
    for (std::string line; std::getline(infile, line);)
    {
        std::stringstream ss(line);
        std::string str;
        std::vector<std::string> lineArray;
        std::vector<double> data;
        while (getline(ss, str, ' '))
        {
            lineArray.push_back(str);
        }
        for (int i = 0; i < lineArray.size(); i++)
        {
            std::stringstream ss;
            double temp_data;
            ss << lineArray[i];
            ss >> temp_data;
            data.push_back(temp_data);
        }
        // if (_Poses.size() > 1000)
        //     continue;
        _Poses.push_back(data);
        std::pair<int, double> tmp_pair;
        tmp_pair.first = _Poses.size() - 1;
        tmp_pair.second = data[0];
        _PoseTimeStamp.push_back(tmp_pair);
    }
    return ret;
}

int Mapping::readSonarWaveData(const std::string filepath)
{
    int ret = -1;
    std::string filenames = filepath + "/ArmyUltra.txt";
    std::ifstream infile(filenames.c_str());
    int count = 0;
    for (std::string line; std::getline(infile, line);)
    {
        std::stringstream ss(line);
        std::string str;
        std::vector<std::string> lineArray;
        std::vector<double> data;
        while (getline(ss, str, ' '))
        {
            lineArray.push_back(str);
        }
        for (int i = 0; i < lineArray.size(); i++)
        {
            std::stringstream ss;
            double temp_data;
            ss << lineArray[i];
            ss >> temp_data;
            data.push_back(temp_data);
        }
        // if (_SonarWaveDatas.size() > 1000)
        //     continue;
        _SonarWaveDatas.push_back(data);
        std::pair<int, double> tmp_pair;
        tmp_pair.first = _SonarWaveDatas.size() - 1;
        tmp_pair.second = data[0];
        _SonarWaveTimeStamp.push_back(tmp_pair);
    }
    return ret;
}

int Mapping::buildMap()
{
    for (int i = 0; i < _SonarWaveDatas.size(); i++)
    {
        std::vector<double> data = _SonarWaveDatas[i];
        double sonarTimeStamp = data[0] * 1e-6;
        int index = timeStampSynchronization(sonarTimeStamp);
        // std::cout<<"i:"<<i<<std::endl;
        // std::cout<<"sonarTimeStamp"<<sonarTimeStamp<<std::fixed<<std::endl;
        // std::cout<<"_Poses[index][0]"<<_Poses[index][0] * 1e-6<<std::fixed<<std::endl;
        // std::cout<<"sonarTimeStamp - _Poses[index][0] * 1e-6:"<<sonarTimeStamp - _Poses[index][0] * 1e-6 <<std::endl;
        Sonar2cloud(SonarIndex::left_front, i, index, _leftFrontCloud);
        Sonar2cloud(SonarIndex::left_back, i, index, _leftBackCloud);
    }
    std::cout << "_leftFrontCloud->points.size():" << _leftFrontCloud->points.size() << std::endl;
    std::cout << "_leftBackCloud->points.size():" << _leftBackCloud->points.size() << std::endl;
    _leftFrontCloud->height = 1;
    _leftFrontCloud->width = _leftFrontCloud->points.size();
    _leftBackCloud->height = 1;
    _leftBackCloud->width = _leftBackCloud->points.size();
    _CloudAll->height = 1;
    _CloudAll->width = _CloudAll->points.size();
    if (_leftFrontCloud->points.size() != 0)
        pcl::io::savePCDFileASCII("leftFrontCloud.pcd", *_leftFrontCloud);
    if (_leftBackCloud->points.size() != 0)
        pcl::io::savePCDFileASCII("leftBackCloud.pcd", *_leftBackCloud);
    if (_CloudAll->points.size() != 0)
    {
        pcl::io::savePCDFileASCII("CloudAll.pcd", *_CloudAll);
        pcl::io::savePLYFileBinary("CloudAll.ply", *_CloudAll);
    }
}

int Mapping::timeStampSynchronization(double sonarWaveTimeStamp)
{

    double pose_stamp = _PoseTimeStamp.front().second * 1e-6;

    int index = _PoseTimeStamp.size() - 1;
    int index_l = 0;
    int index_r = 0;
    bool picked = false;
    bool picked_l = false;
    bool picked_r = false;

    // 1. pick index
    for (int i = index; i >= 0; i--)
    {

        double timestamp = _PoseTimeStamp.at(i).second * 1e-6;
        if (timestamp - sonarWaveTimeStamp > 0)
        {
            picked_r = true;
            index_r = i;
        }
        if (timestamp - sonarWaveTimeStamp <= 0)
        {
            picked_l = true;
            index_l = i;
        }

        if (picked_r && picked_l)
        {
            if ((_PoseTimeStamp.at(index_r).second * 1e-6 - sonarWaveTimeStamp) >= (sonarWaveTimeStamp - _PoseTimeStamp.at(index_l).second * 1e-6))
            {

                index = index_l;
                break;
            }
            else
            {

                index = index_r;
                break;
            }
        }
    }
    if (index_r && !picked_l)
        index = index_r;

    // // 排除数据
    // if (picked_r && picked_l)
    // {
    //     for (int i = 0; i <= index; i++)
    //     {
    //         _PoseTimeStamp.pop_front();
    //     }
    // }
    return _PoseTimeStamp.at(index).first;
}

int Mapping::Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    float fov_rad = 0.0, sonar_base_x = 0.0, sonar_base_y = 0.0, sonar_base_yaw = 0.0;
    float length;
    if (index == SonarIndex::left_front)
    {
        fov_rad = _leftFrontFovRad;
        sonar_base_x = _leftFrontX;
        sonar_base_y = _leftFronty;
        sonar_base_yaw = _leftFrontyaw;
        length = _SonarWaveDatas[indexSonar][1];
    }
    else if (index == SonarIndex::left_back)
    {
        fov_rad = _leftBackFovRad;
        sonar_base_x = _leftBackX;
        sonar_base_y = _leftBackY;
        sonar_base_yaw = _leftBackYaw;
        length = _SonarWaveDatas[indexSonar][2];
    }
    float half_fov = fov_rad / 2.0;
    float theta_rad = 0.0 / 180.0 * M_PI;

    // sonar坐标系的数据
    float sonar_x, sonar_y, sonar_z;
    sonar_z = 0.0;
    sonar_x = length * cos(theta_rad);
    sonar_y = length * sin(theta_rad);

    // sonar坐标系转base_link
    float base_x = sonar_base_x + (sonar_x * cos(sonar_base_yaw) - sonar_y * sin(sonar_base_yaw));
    float base_y = sonar_base_y + (sonar_x * sin(sonar_base_yaw) + sonar_y * cos(sonar_base_yaw));
    Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond cur_Q_ = Eigen::AngleAxisd(_Poses[indexPose][6], Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(_Poses[indexPose][5], Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(_Poses[indexPose][4], Eigen::Vector3d::UnitX());
    cur_Q_.normalize();
    T_wc.block<3, 3>(0, 0) = cur_Q_.toRotationMatrix();
    T_wc.block<3, 1>(0, 3) = Eigen::Vector3d(_Poses[indexPose][1], _Poses[indexPose][2], _Poses[indexPose][3]);
    Eigen::Vector4d p_ = Eigen::Vector4d(base_x, base_y, 0, 1);
    Eigen::Vector4d p_w = T_wc * p_;
    Eigen::Vector3d p_w_3 = Eigen::Vector3d(p_w[0], p_w[1], p_w[2]);
    if (index == SonarIndex::left_front)
    {
        pcl::PointXYZI p;
        p.x = p_w_3[0];
        p.y = p_w_3[1];
        p.z = p_w_3[2];
        p.intensity = 0;
        cloud->points.push_back(p);
        pcl::PointXYZRGB p_rgb;
        p_rgb.x = p_w_3[0];
        p_rgb.y = p_w_3[1];
        p_rgb.z = p_w_3[2];
        p_rgb.r = 255;
        p_rgb.g = 0;
        p_rgb.b = 0;
        _CloudAll->points.push_back(p_rgb);
    }
    else if (index == SonarIndex::left_back)
    {
        pcl::PointXYZI p;
        p.x = p_w_3[0];
        p.y = p_w_3[1];
        p.z = p_w_3[2];
        p.intensity = 0;
        cloud->points.push_back(p);
        pcl::PointXYZRGB p_rgb;
        p_rgb.x = p_w_3[0];
        p_rgb.y = p_w_3[1];
        p_rgb.z = p_w_3[2];
        p_rgb.r = 0;
        p_rgb.g = 255;
        p_rgb.b = 0;
        _CloudAll->points.push_back(p_rgb);
    }
    return 1;
}
int Mapping::Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Matrix3d R12, Eigen::Vector3d t12)
{
    Eigen::Matrix4d T_12 = Eigen::Matrix4d::Identity();
    T_12.block<3, 3>(0, 0) = R12;
    T_12.block<3, 1>(0, 3) = t12;
    float fov_rad = 0.0, sonar_base_x = 0.0, sonar_base_y = 0.0, sonar_base_yaw = 0.0;
    float length;
    if (index == SonarIndex::left_front)
    {
        fov_rad = _leftFrontFovRad;
        sonar_base_x = _leftFrontX;
        sonar_base_y = _leftFronty;
        sonar_base_yaw = _leftFrontyaw;
        length = _SonarWaveDatas[indexSonar][1];
    }
    else if (index == SonarIndex::left_back)
    {
        fov_rad = _leftBackFovRad;
        sonar_base_x = _leftBackX;
        sonar_base_y = _leftBackY;
        sonar_base_yaw = _leftBackYaw;
        length = _SonarWaveDatas[indexSonar][2];
    }
    float half_fov = fov_rad / 2.0;
    float theta_rad = 0.0 / 180.0 * M_PI;

    // sonar坐标系的数据
    float sonar_x, sonar_y, sonar_z;
    sonar_z = 0.0;
    sonar_x = length * cos(theta_rad);
    sonar_y = length * sin(theta_rad);
    Eigen::Vector4d sonar_p(sonar_x, sonar_y, sonar_z, 1);

    Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond cur_Q_ = Eigen::AngleAxisd(_Poses[indexPose][6], Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(_Poses[indexPose][5], Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(_Poses[indexPose][4], Eigen::Vector3d::UnitX());
    cur_Q_.normalize();
    T_wc.block<3, 3>(0, 0) = cur_Q_.toRotationMatrix();
    T_wc.block<3, 1>(0, 3) = Eigen::Vector3d(_Poses[indexPose][1], _Poses[indexPose][2], _Poses[indexPose][3]);

    Eigen::Vector3d p_w_3;

    if (_leftBackBase)
    {
        Eigen::Matrix4d T_base_back;
        T_base_back.setIdentity();
        Eigen::Quaterniond q_base_back = Eigen::AngleAxisd(_leftBackYaw, Eigen::Vector3d::UnitZ()) *
                                         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        q_base_back.normalize();
        T_base_back.block<3, 3>(0, 0) = q_base_back.toRotationMatrix();
        T_base_back.block<3, 1>(0, 3) = Eigen::Vector3d(_leftBackX, _leftBackY, 0);

        Eigen::Matrix4d T_w_base = T_wc;
        Eigen::Matrix4d T_w_back = T_w_base * T_base_back;
        Eigen::Quaterniond q_w_back(T_w_back.block<3, 3>(0, 0));
        Eigen::Vector3d t_w_back(T_w_back.block<3, 1>(0, 3));
        Eigen::Vector4d p_w_4;
        p_w_4 = T_w_back * T_12 * sonar_p;
        p_w_3.x() = p_w_4.x();
        p_w_3.y() = p_w_4.y();
        p_w_3.z() = p_w_4.z();
    }
    else
    {
        Eigen::Matrix4d T_base_front;
        T_base_front.setIdentity();
        Eigen::Quaterniond q_base_front = Eigen::AngleAxisd(_leftFrontyaw, Eigen::Vector3d::UnitZ()) *
                                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        T_base_front.block<3, 3>(0, 0) = q_base_front.toRotationMatrix();
        T_base_front.block<3, 1>(0, 3) = Eigen::Vector3d(_leftFrontX, _leftFronty, 0);

        Eigen::Matrix4d T_w_base = T_wc;
        Eigen::Matrix4d T_w_front = T_w_base * T_base_front;
        Eigen::Quaterniond q_w_front(T_w_front.block<3, 3>(0, 0));
        Eigen::Vector3d t_w_front(T_w_front.block<3, 1>(0, 3));
        Eigen::Vector4d p_w_4;
        p_w_4 = T_w_front * T_12 * sonar_p;
        p_w_3.x() = p_w_4.x();
        p_w_3.y() = p_w_4.y();
        p_w_3.z() = p_w_4.z();

        Eigen::Matrix4d T_base_back = T_base_front * T_12;
        Eigen::Vector3d ypr;
        Eigen::Matrix3d R_base_back = T_base_back.block<3, 3>(0, 0);
        ypr = R_base_back.eulerAngles(2, 1, 0);
        Eigen::Vector3d t_base_back = T_base_back.block<3, 1>(0, 3);
        //  std::cout<<"R_base_back:"<<ypr<<std::endl;
        //  std::cout<<"t_base_back:"<<t_base_back<<std::endl;
        //  std::cout<<"R12:"<<R12<<std::endl;
    }

    if (index == SonarIndex::left_front)
    {
        pcl::PointXYZI p;
        p.x = p_w_3[0];
        p.y = p_w_3[1];
        p.z = p_w_3[2];
        p.intensity = 0;
        cloud->points.push_back(p);
        pcl::PointXYZRGB p_rgb;
        p_rgb.x = p_w_3[0];
        p_rgb.y = p_w_3[1];
        p_rgb.z = p_w_3[2];
        p_rgb.r = 255;
        p_rgb.g = 0;
        p_rgb.b = 0;
        _CloudAll->points.push_back(p_rgb);
    }
    else if (index == SonarIndex::left_back)
    {
        pcl::PointXYZI p;
        p.x = p_w_3[0];
        p.y = p_w_3[1];
        p.z = p_w_3[2];
        p.intensity = 0;
        cloud->points.push_back(p);
        pcl::PointXYZRGB p_rgb;
        p_rgb.x = p_w_3[0];
        p_rgb.y = p_w_3[1];
        p_rgb.z = p_w_3[2];
        p_rgb.r = 0;
        p_rgb.g = 255;
        p_rgb.b = 0;
        _CloudAll->points.push_back(p_rgb);
    }
    return 1;
}
void Mapping::buildMultiFrame()
{
    for (int i = 0; i < _SonarWaveDatas.size(); i++)
    {
        std::vector<double> data = _SonarWaveDatas[i];
        double sonarTimeStamp = data[0] * 1e-6;
        int index = timeStampSynchronization(sonarTimeStamp);
        Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond cur_Q_ = Eigen::AngleAxisd(_Poses[index][6], Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(_Poses[index][5], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(_Poses[index][4], Eigen::Vector3d::UnitX());
        cur_Q_.normalize();
        T_wc.block<3, 3>(0, 0) = cur_Q_.toRotationMatrix();
        T_wc.block<3, 1>(0, 3) = Eigen::Vector3d(_Poses[index][1], _Poses[index][2], _Poses[index][3]);
        _p_sonarindedx_poseindex.push_back(std::pair<int, int>(i, index));
        _p_sonarindex_pose.push_back(std::pair<int, Eigen::Matrix4d>(i, T_wc));
    }
    double distance_threshold = 0.5;
    bool first_pose = false;
    Eigen::Matrix4d first_T;
    clusterPoses(distance_threshold);

    if (_clustered_poses.size() > 1)
    {
        double travelDistance = 0.0;
        std::vector<std::pair<int, Eigen::Matrix4d>> &lastGroup = _clustered_poses.back();

        // 计算最后一组中位姿依次行走的距离
        for (size_t i = 1; i < lastGroup.size(); ++i)
        {
            travelDistance += calculateDistance(lastGroup[i - 1].second, lastGroup[i].second);
        }

        // 如果累计行走的距离小于阈值，则合并至前一组
        std::cout << "travelDistance:" << travelDistance << std::endl;
        if (travelDistance < distance_threshold * 0.7)
        {
            std::cout << "travelDistance < distance_threshold * 0.5" << std::endl;
            std::vector<std::pair<int, Eigen::Matrix4d>> &secondLastGroup = _clustered_poses[_clustered_poses.size() - 2];
            secondLastGroup.insert(secondLastGroup.end(), lastGroup.begin(), lastGroup.end());
            _clustered_poses.pop_back(); // 移除最后一组
        }
    }

    // std::cout << "_p_sonarindex_pose: " << _p_sonarindex_pose.size() << std::endl;
    // int num_cluster;
    // for (int i = 0; i < _clustered_poses.size(); ++i)
    // {
    //     std::cout << "Cluster " << i << " has " << _clustered_poses[i].size() << " poses." << std::endl;
    //     num_cluster += _clustered_poses[i].size();
    // }
    // std::cout << "num_cluster:" << num_cluster << std::endl;
    multiFrameCombined();
}
void Mapping::multiFrameCombined()
{

    for (int i = 0; i < _clustered_poses.size(); i++)
    {
        std::vector<std::pair<int, Eigen::Matrix4d>> &cluster = _clustered_poses[i];
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (int j = 0; j < cluster.size(); j++)
        {
            int index = cluster[j].first;
            Eigen::Matrix4d T_wc = cluster[j].second;

            float fov_rad = 0.0, sonar_base_x = 0.0, sonar_base_y = 0.0, sonar_base_yaw = 0.0;
            float length;
            fov_rad = _leftFrontFovRad;
            sonar_base_x = _leftFrontX;
            sonar_base_y = _leftFronty;
            sonar_base_yaw = _leftFrontyaw;
            length = _SonarWaveDatas[index][1];
            float half_fov = fov_rad / 2.0;
            float theta_rad = 0.0 / 180.0 * M_PI;

            // sonar坐标系的数据
            float sonar_x, sonar_y, sonar_z;
            sonar_z = 0.0;
            sonar_x = length * cos(theta_rad);
            sonar_y = length * sin(theta_rad);
            Eigen::Vector4d sonar_p(sonar_x, sonar_y, sonar_z, 1);
            Eigen::Matrix4d T_base_front;
            T_base_front.setIdentity();
            Eigen::Quaterniond q_base_front = Eigen::AngleAxisd(_leftFrontyaw, Eigen::Vector3d::UnitZ()) *
                                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
            T_base_front.block<3, 3>(0, 0) = q_base_front.toRotationMatrix();
            T_base_front.block<3, 1>(0, 3) = Eigen::Vector3d(_leftFrontX, _leftFronty, 0);

            Eigen::Matrix4d T_w_base = T_wc;
            Eigen::Matrix4d T_w_front = T_w_base * T_base_front;
            Eigen::Quaterniond q_w_front(T_w_front.block<3, 3>(0, 0));
            Eigen::Vector3d t_w_front(T_w_front.block<3, 1>(0, 3));
            Eigen::Vector4d p_w_4;
            Eigen::Vector4d p_0; // 转到第一帧坐标系下
            p_w_4 = T_w_front * sonar_p;
            p_0 = _clustered_poses[i][0].second.inverse() * p_w_4;
            pcl::PointXYZI p;
            p.x = p_0[0];
            p.y = p_0[1];
            p.z = p_0[2];
            cloud->points.push_back(p);
        }
        // cloud->height = 1;
        // cloud->width = cloud->points.size();
        // if (cloud->points.size() != 0)
        // {
        //     pcl::io::savePLYFileBinary(std::to_string(i) + ".ply", *cloud);
        // }
        _p_cloud_pose.push_back(std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::Matrix4d>(cloud, _clustered_poses[i][0].second));
    }
    // 构造多帧点云使得每帧之间有重合
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::Matrix4d>> p_cloud_pose_new;
    for (int i = 0; i < _p_cloud_pose.size() - 1; i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*_p_cloud_pose[i].first, *cloud1);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*_p_cloud_pose[i + 1].first, *cloud2);
        Eigen::Matrix4d T1 = _p_cloud_pose[i].second;
        Eigen::Matrix4d T2 = _p_cloud_pose[i + 1].second;
        pcl::transformPointCloud(*cloud1, *cloud1, T1);
        pcl::transformPointCloud(*cloud2, *cloud2, T2);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_combine(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_combine->points.insert(cloud_combine->points.end(), cloud1->points.begin(), cloud1->points.end());
        cloud_combine->points.insert(cloud_combine->points.end(), cloud2->points.begin(), cloud2->points.end());
        Eigen::Matrix4d T = T1.inverse();
        pcl::transformPointCloud(*cloud_combine, *cloud_combine, T);
        // cloud_combine->height = 1;
        // cloud_combine->width = cloud_combine->points.size();
        // if (cloud_combine->points.size() != 0)
        // {
        //     pcl::io::savePLYFileBinary(std::to_string(i) + ".ply", *cloud_combine);
        // }
        p_cloud_pose_new.push_back(std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::Matrix4d>(cloud_combine, T1));
    }
    _p_cloud_pose = p_cloud_pose_new;
}

double Mapping::calculateDistance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2)
{
    return std::sqrt((pose1(0, 3) - pose2(0, 3)) * (pose1(0, 3) - pose2(0, 3)) +
                     (pose1(1, 3) - pose2(1, 3)) * (pose1(1, 3) - pose2(1, 3)) +
                     (pose1(2, 3) - pose2(2, 3)) * (pose1(2, 3) - pose2(2, 3)));
}
void Mapping::clusterPoses(double maxDistance)
{
    std::vector<bool> clustered(_p_sonarindex_pose.size(), false); // 标记位姿是否已经被分到某个簇中

    for (size_t i = 0; i < _p_sonarindex_pose.size(); ++i)
    {
        // 跳过已经聚类的位姿
        if (clustered[i])
            continue;

        std::vector<std::pair<int, Eigen::Matrix4d>> cluster;
        cluster.push_back(_p_sonarindex_pose[i]);
        clustered[i] = true;
        int timestamp_index = _p_sonarindedx_poseindex[i].second;
        double timestamp = _Poses[timestamp_index][0] * 1e-6;

        // 对后续位姿进行检查，判断是否与当前位姿是同一簇的一部分
        for (size_t j = i + 1; j < _p_sonarindex_pose.size(); ++j)
        {
            if (!clustered[j] && calculateDistance(_p_sonarindex_pose[i].second, _p_sonarindex_pose[j].second) <= maxDistance &&
                std::abs(timestamp - _Poses[_p_sonarindedx_poseindex[j].second][0] * 1e-6) <= 20)
            {
                cluster.push_back(_p_sonarindex_pose[j]);
                clustered[j] = true;
            }
        }

        _clustered_poses.push_back(cluster);
    }
}

void Mapping::map()
{
    for (int i = 0; i < _p_cloud_pose.size(); i++)
    {
        if (_keyframes.empty())
        {
            _keyframes.push_back(_p_cloud_pose[i]);
            continue;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = _p_cloud_pose[i].first;
        Eigen::Matrix4d T = _p_cloud_pose[i].second;
        pcl::PointCloud<pcl::PointXYZI>::Ptr localMap(new pcl::PointCloud<pcl::PointXYZI>);
        for (int j = 0; j < _keyframes.size(); j++)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr keyframe = _keyframes[j].first;
            Eigen::Matrix4d T_keyframe = _keyframes[j].second;
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*keyframe, *transformedCloud, T_keyframe);
            *localMap += *transformedCloud;
        }
        _m_iNdt.setResolution(0.1);
        _m_iNdt.setMaximumIterations(100);
        _m_iNdt.setTransformationEpsilon(0.00001);
        _m_iNdt.setStepSize(0.01);
        _m_iNdt.setInputTarget(localMap);
        _m_iNdt.setInputSource(cloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>());
        Eigen::Matrix4f pose;
        pose = T.cast<float>();
        _m_iNdt.align(*out, pose);
        pose = _m_iNdt.getFinalTransformation();

        float closest_d = std::numeric_limits<float>::infinity();
        for (const auto &k : _keyframes)
        {
            // calculate distance between current pose and pose in keyframes
            double delta_d = sqrt(pow(pose(0,3) - k.second(0,3), 2) + pow(pose(1,3)- k.second(1,3), 2) +
                                  pow(pose(2,3) - k.second(2,3), 2));
            // store into variable
            if (delta_d < closest_d)
            {
                closest_d = delta_d;
            }
        }
        std::cout << "closest_d:" << closest_d << std::endl;
        if(closest_d > 0.4)
        {
            _keyframes.push_back(std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::Matrix4d>(cloud, pose.cast<double>()));
        }

        
        std::cout << "pose:" << pose << std::endl;
        std::cout << "T:" << T << std::endl;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
    for (int j = 0; j < _keyframes.size(); j++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr keyframe = _keyframes[j].first;
        Eigen::Matrix4d T_keyframe = _keyframes[j].second;
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*keyframe, *transformedCloud, T_keyframe);
        *out += *transformedCloud;
    }
    out->height = 1;
    out->width = out->points.size();
    if (out->points.size() != 0)
    {
        pcl::io::savePLYFileBinary("out.ply", *out);
    }
}
