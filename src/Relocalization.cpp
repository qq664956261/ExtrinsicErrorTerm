#include <ExtrinsicErrorTerm/Relocalization.h>
Relocalization::Relocalization()
{
    _leftFrontCloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
    _leftBackCloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
    _CloudAll.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
}
Relocalization::~Relocalization()
{
}

int Relocalization::readPose(const std::string filepath, std::vector<std::vector<double>> &Poses, std::deque<std::pair<int, double>> &PoseTimeStamp)
{
    std::cout << "readPose" << std::endl;
    int ret = -1;
    std::string filenames = filepath;
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
        Poses.push_back(data);
        std::pair<int, double> tmp_pair;
        tmp_pair.first = Poses.size() - 1;
        tmp_pair.second = data[0];
        PoseTimeStamp.push_back(tmp_pair);
    }
    return ret;
}
void Relocalization::setPoses(std::vector<std::vector<double>> Poses, std::deque<std::pair<int, double>> PoseTimeStamp)
{
    _Poses = Poses;
    _PoseTimeStamp = PoseTimeStamp;
}
void Relocalization::setPosesRelocSource(std::vector<std::vector<double>> Poses, std::deque<std::pair<int, double>> PoseTimeStamp)
{
    _PosesRelocSource = Poses;
    _PoseTimeStampRelocSource = PoseTimeStamp;
}

int Relocalization::readSonarWaveData(const std::string filepath, std::vector<std::vector<double>> &SonarWaveDatas, std::deque<std::pair<int, double>> &SonarWaveTimeStamp)
{
    std::cout << "readSonarWaveData" << std::endl;
    int ret = -1;
    std::string filenames = filepath;
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
        SonarWaveDatas.push_back(data);
        std::pair<int, double> tmp_pair;
        tmp_pair.first = SonarWaveDatas.size() - 1;
        tmp_pair.second = data[0];
        SonarWaveTimeStamp.push_back(tmp_pair);
    }

    return ret;
}

void Relocalization::setSonarData(std::vector<std::vector<double>> SonarWaveDatas, std::deque<std::pair<int, double>> SonarWaveTimeStamp)
{
    _SonarWaveDatas = SonarWaveDatas;
    _SonarWaveTimeStamp = SonarWaveTimeStamp;
}
void Relocalization::setSonarDataRelocSource(std::vector<std::vector<double>> SonarWaveDatas, std::deque<std::pair<int, double>> SonarWaveTimeStamp)
{
    _SonarWaveDatasRelocSource = SonarWaveDatas;
    _SonarWaveTimeStampRelocSource = SonarWaveTimeStamp;
}

int Relocalization::buildMap()
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
    // _leftFrontCloud->height = 1;
    // _leftFrontCloud->width = _leftFrontCloud->points.size();
    // _leftBackCloud->height = 1;
    // _leftBackCloud->width = _leftBackCloud->points.size();
    // _CloudAll->height = 1;
    // _CloudAll->width = _CloudAll->points.size();
    if (_leftFrontCloud->points.size() != 0)
        mypcl::savePLYFileBinary("leftFrontCloud.ply", *_leftFrontCloud);
    if (_leftBackCloud->points.size() != 0)
        mypcl::savePLYFileBinary("leftBackCloud.ply", *_leftBackCloud);
    if (_CloudAll->points.size() != 0)
    {
        mypcl::savePLYFileBinary("CloudAll.ply", *_CloudAll);
    }
}

int Relocalization::timeStampSynchronization(double sonarWaveTimeStamp)
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

int Relocalization::timeStampSynchronizationRelocSource(double sonarWaveTimeStamp)
{
    double pose_stamp = _PoseTimeStampRelocSource.front().second * 1e-6;
    int index = _PoseTimeStampRelocSource.size() - 1;
    int index_l = 0;
    int index_r = 0;
    bool picked = false;
    bool picked_l = false;
    bool picked_r = false;

    // 1. pick index
    for (int i = index; i >= 0; i--)
    {

        double timestamp = _PoseTimeStampRelocSource.at(i).second * 1e-6;
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
            if ((_PoseTimeStampRelocSource.at(index_r).second * 1e-6 - sonarWaveTimeStamp) >= (sonarWaveTimeStamp - _PoseTimeStampRelocSource.at(index_l).second * 1e-6))
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

    return _PoseTimeStampRelocSource.at(index).first;
}

int Relocalization::Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud)
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
        mypcl::PointXYZI p;
        p.x = p_w_3[0];
        p.y = p_w_3[1];
        p.z = p_w_3[2];
        p.intensity = 0;
        cloud->points.push_back(p);
        mypcl::PointXYZI p_rgb;
        p_rgb.x = p_w_3[0];
        p_rgb.y = p_w_3[1];
        p_rgb.z = p_w_3[2];
        // p_rgb.r = 255;
        // p_rgb.g = 0;
        // p_rgb.b = 0;
        _CloudAll->points.push_back(p_rgb);
    }
    else if (index == SonarIndex::left_back)
    {
        mypcl::PointXYZI p;
        p.x = p_w_3[0];
        p.y = p_w_3[1];
        p.z = p_w_3[2];
        p.intensity = 0;
        cloud->points.push_back(p);
        mypcl::PointXYZI p_rgb;
        p_rgb.x = p_w_3[0];
        p_rgb.y = p_w_3[1];
        p_rgb.z = p_w_3[2];
        // p_rgb.r = 0;
        // p_rgb.g = 255;
        // p_rgb.b = 0;
        _CloudAll->points.push_back(p_rgb);
    }
    return 1;
}
int Relocalization::Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud, Eigen::Matrix3d R12, Eigen::Vector3d t12)
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
        mypcl::PointXYZI p;
        p.x = p_w_3[0];
        p.y = p_w_3[1];
        p.z = p_w_3[2];
        p.intensity = 0;
        cloud->points.push_back(p);
        mypcl::PointXYZI p_rgb;
        p_rgb.x = p_w_3[0];
        p_rgb.y = p_w_3[1];
        p_rgb.z = p_w_3[2];
        // p_rgb.r = 255;
        // p_rgb.g = 0;
        // p_rgb.b = 0;
        _CloudAll->points.push_back(p_rgb);
    }
    else if (index == SonarIndex::left_back)
    {
        mypcl::PointXYZI p;
        p.x = p_w_3[0];
        p.y = p_w_3[1];
        p.z = p_w_3[2];
        p.intensity = 0;
        cloud->points.push_back(p);
        mypcl::PointXYZI p_rgb;
        p_rgb.x = p_w_3[0];
        p_rgb.y = p_w_3[1];
        p_rgb.z = p_w_3[2];
        // p_rgb.r = 0;
        // p_rgb.g = 255;
        // p_rgb.b = 0;
        _CloudAll->points.push_back(p_rgb);
    }
    return 1;
}
void Relocalization::buildMultiFrame()
{
    std::cout << "buildMultiFrame" << std::endl;
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
    std::cout << "_p_sonarindedx_poseindex.size():" << _p_sonarindedx_poseindex.size() << std::endl;
    bool first_pose = false;
    Eigen::Matrix4d first_T;
    clusterPoses(_distance_threshold);

    if (_clustered_poses.size() > 1)
    {
        double travelDistance = 0.0;
        std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>> &lastGroup = _clustered_poses.back();

        // 计算最后一组中位姿依次行走的距离
        for (size_t i = 1; i < lastGroup.size(); ++i)
        {
            travelDistance += calculateDistance(lastGroup[i - 1].second.second, lastGroup[i].second.second);
        }

        // 如果累计行走的距离小于阈值，则合并至前一组
        std::cout << "travelDistance:" << travelDistance << std::endl;
        if (travelDistance < _distance_threshold * 0.7)
        {
            std::cout << "travelDistance < distance_threshold * 0.7" << std::endl;
            std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>> &secondLastGroup = _clustered_poses[_clustered_poses.size() - 2];
            secondLastGroup.insert(secondLastGroup.end(), lastGroup.begin(), lastGroup.end());
            _clustered_poses.pop_back(); // 移除最后一组
        }
    }

    std::cout << "multiFrameCombined" << std::endl;
    multiFrameCombined();
}
void Relocalization::multiFrameCombined()
{

    for (int i = 0; i < _clustered_poses.size(); i++)
    {
        std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>> &cluster = _clustered_poses[i];
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud(new mypcl::PointCloud<mypcl::PointXYZI>);
        for (int j = 0; j < cluster.size(); j++)
        {
            int index = cluster[j].first;
            Eigen::Matrix4d T_wc = cluster[j].second.second;

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
            p_0 = _clustered_poses[i][0].second.second.inverse() * p_w_4;
            mypcl::PointXYZI p;
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
        std::pair<mypcl::PointCloud<mypcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>> p_cloud_time_pose;
        // p_cloud_time_pose.first = cloud;
        p_cloud_time_pose.first.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
        mypcl::copyPointCloud(*cloud, *p_cloud_time_pose.first);
        p_cloud_time_pose.second.first = _clustered_poses[i][0].second.first;
        p_cloud_time_pose.second.second = _clustered_poses[i][0].second.second;
        _p_cloud_pose.push_back(p_cloud_time_pose);
    }
    // 新建 _p_cloud_pose_combined 来保存组合后的点云和位姿信息
    std::vector<std::pair<mypcl::PointCloud<mypcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>>> _p_cloud_pose_combined;

    // 准备将 _p_cloud_pose 分成若干组，每组10个
    for (size_t i = 0; i < _p_cloud_pose.size(); i++)
    {
        int count = 0;
        auto combined_cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();

        // 定义组内时间戳和位姿的初始值
        double earliest_timestamp = _p_cloud_pose[i].second.first;
        Eigen::Matrix4d reference_pose = _p_cloud_pose[i].second.second;
        int j = i;
        Eigen::Matrix4d T1;
        while (count < _combineFrame && j < _p_cloud_pose.size())
        {
            // 累加当前组内所有帧的点到 combined_cloud
            auto new_cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
            mypcl::copyPointCloud(*(_p_cloud_pose[j].first), *new_cloud);
            Eigen::Matrix4d new_pose = _p_cloud_pose[j].second.second;
            if (count == 0)
            {
                T1 = new_pose;
            }
            mypcl::transformPointCloud(*new_cloud, *new_cloud, new_pose.cast<float>());

            *combined_cloud += *(new_cloud);
            j++;
            count++;
        }

        mypcl::transformPointCloud(*combined_cloud, *combined_cloud, T1.inverse().cast<float>());

        // 如果这是最后一组且它不足10帧，则与之前的点云合并
        if (count < _combineFrame / 2 && !_p_cloud_pose_combined.empty())
        {
            *(_p_cloud_pose_combined.back().first) += *combined_cloud;
        }
        else
        {
            // 存储当前组的组合点云和第一帧的时间戳及位姿
            std::pair<double, Eigen::Matrix4d> time_pose = std::make_pair(earliest_timestamp, reference_pose);
            _p_cloud_pose_combined.push_back(std::make_pair(combined_cloud, time_pose));
        }

        if (combined_cloud->points.size() != 0)
        {
            mypcl::savePLYFileBinary(std::to_string(i) + ".ply", *combined_cloud);
        }
    }

    _p_cloud_pose = _p_cloud_pose_combined;
}

double Relocalization::calculateDistance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2)
{
    return std::sqrt((pose1(0, 3) - pose2(0, 3)) * (pose1(0, 3) - pose2(0, 3)) +
                     (pose1(1, 3) - pose2(1, 3)) * (pose1(1, 3) - pose2(1, 3)));
}
void Relocalization::clusterPoses(double maxDistance)
{
    std::vector<bool> clustered(_p_sonarindex_pose.size(), false); // 标记位姿是否已经被分到某个簇中

    for (size_t i = 0; i < _p_sonarindex_pose.size(); ++i)
    {
        // 跳过已经聚类的位姿
        if (clustered[i])
            continue;

        std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>> cluster;
        std::pair<int, std::pair<double, Eigen::Matrix4d>> temp_pair;
        temp_pair.first = _p_sonarindex_pose[i].first;
        temp_pair.second = std::pair<double, Eigen::Matrix4d>(_Poses[_p_sonarindedx_poseindex[i].second][0], _p_sonarindex_pose[i].second);
        cluster.push_back(temp_pair);
        clustered[i] = true;
        int timestamp_index = _p_sonarindedx_poseindex[i].second;
        double timestamp = _Poses[timestamp_index][0] * 1e-6;

        // 对后续位姿进行检查，判断是否与当前位姿是同一簇的一部分
        for (size_t j = i + 1; j < _p_sonarindex_pose.size(); ++j)
        {
            if (!clustered[j] && calculateDistance(_p_sonarindex_pose[i].second, _p_sonarindex_pose[j].second) <= maxDistance &&
                std::abs(timestamp - _Poses[_p_sonarindedx_poseindex[j].second][0] * 1e-6) <= 40)
            {
                temp_pair.first = _p_sonarindex_pose[j].first;
                temp_pair.second = std::pair<double, Eigen::Matrix4d>(_Poses[_p_sonarindedx_poseindex[j].second][0], _p_sonarindex_pose[j].second);
                cluster.push_back(temp_pair);
                clustered[j] = true;
            }
        }

        _clustered_poses.push_back(cluster);
    }
}
void Relocalization::buildRelocSource()
{
    for (int i = 0; i < _SonarWaveDatasRelocSource.size(); i++)
    {
        std::vector<double> data = _SonarWaveDatasRelocSource[i];
        double sonarTimeStamp = data[0] * 1e-6;
        int index = timeStampSynchronizationRelocSource(sonarTimeStamp);
        Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond cur_Q_ = Eigen::AngleAxisd(_PosesRelocSource[index][6], Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(_PosesRelocSource[index][5], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(_PosesRelocSource[index][4], Eigen::Vector3d::UnitX());
        cur_Q_.normalize();
        T_wc.block<3, 3>(0, 0) = cur_Q_.toRotationMatrix();
        T_wc.block<3, 1>(0, 3) = Eigen::Vector3d(_PosesRelocSource[index][1], _PosesRelocSource[index][2], _PosesRelocSource[index][3]);
        _p_sonarindedx_poseindex_reloc.push_back(std::pair<int, int>(i, index));
        _p_sonarindex_pose_reloc.push_back(std::pair<int, Eigen::Matrix4d>(i, T_wc));
    }

    while (_sourceStartIndex > _p_sonarindex_pose_reloc.size())
    {
        _sourceStartIndex = _sourceStartIndex / 2;
    }

    std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>> cluster;
    std::pair<int, std::pair<double, Eigen::Matrix4d>> temp_pair;
    temp_pair.first = _p_sonarindex_pose_reloc[_sourceStartIndex].first;
    temp_pair.second = std::pair<double, Eigen::Matrix4d>(_PosesRelocSource[_p_sonarindedx_poseindex_reloc[_sourceStartIndex].second][0], _p_sonarindex_pose_reloc[_sourceStartIndex].second);
    cluster.push_back(temp_pair);
    int timestamp_index = _p_sonarindedx_poseindex_reloc[_sourceStartIndex].second;
    double timestamp = _PosesRelocSource[timestamp_index][0] * 1e-6;

    // 对后续位姿进行检查，判断是否与当前位姿是同一簇的一部分
    for (size_t j = _sourceStartIndex + 1; j < _p_sonarindex_pose_reloc.size(); ++j)
    {
        if (calculateDistance(_p_sonarindex_pose_reloc[_sourceStartIndex].second, _p_sonarindex_pose_reloc[j].second) <= _distance_threshold * _combineFrame &&
            std::abs(timestamp - _PosesRelocSource[_p_sonarindedx_poseindex_reloc[j].second][0] * 1e-6) <= 40)
        {
            temp_pair.first = _p_sonarindex_pose_reloc[j].first;
            temp_pair.second = std::pair<double, Eigen::Matrix4d>(_PosesRelocSource[_p_sonarindedx_poseindex_reloc[j].second][0], _p_sonarindex_pose_reloc[j].second);
            cluster.push_back(temp_pair);
        }
    }
    mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud(new mypcl::PointCloud<mypcl::PointXYZI>);
    for (int j = 0; j < cluster.size(); j++)
    {
        int index = cluster[j].first;
        Eigen::Matrix4d T_wc = cluster[j].second.second;

        float fov_rad = 0.0, sonar_base_x = 0.0, sonar_base_y = 0.0, sonar_base_yaw = 0.0;
        float length;
        fov_rad = _leftFrontFovRad;
        sonar_base_x = _leftFrontX;
        sonar_base_y = _leftFronty;
        sonar_base_yaw = _leftFrontyaw;
        length = _SonarWaveDatasRelocSource[index][1];
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
        p_0 = cluster[0].second.second.inverse() * p_w_4;
        mypcl::PointXYZI p;
        p.x = p_0[0];
        p.y = p_0[1];
        p.z = p_0[2];
        cloud->points.push_back(p);
    }
    _reloc_source.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
    mypcl::copyPointCloud(*cloud, *_reloc_source);
    if (cloud->points.size() != 0)
    {
        mypcl::savePLYFileBinary("source.ply", *cloud);
    }
}
void Relocalization::reloc()
{
    buildRelocSource();
    for (int i = 0; i < _p_cloud_pose.size(); i++)
    {
        auto cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
        mypcl::copyPointCloud(*(_p_cloud_pose[i].first), *cloud);
        _scManager.makeAndSaveScancontextAndKeys(*cloud);
    }
    _scManager.makeAndSaveScancontextAndKeys(*_reloc_source);
    auto detectResult = _scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
    int SCclosestHistoryFrameID = detectResult.first;
    const int prev_node_idx = SCclosestHistoryFrameID;
    const int curr_node_idx = _p_cloud_pose.size(); // because cpp starts 0 and ends n-1
    auto target = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
    std::cout << "SCclosestHistoryFrameID:" << SCclosestHistoryFrameID << std::endl;
    mypcl::copyPointCloud(*(_p_cloud_pose[SCclosestHistoryFrameID].first), *target);
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd rotation_vector(detectResult.second, Eigen::Vector3d(0, 0, 1));
    rotation_matrix = rotation_vector.toRotationMatrix();
    T.block<3, 3>(0, 0) = rotation_matrix;
    // mypcl::transformPointCloud(*target, *target, T.cast<float>());
    mypcl::savePLYFileBinary("target.ply", *target);
    auto source_tran = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
    mypcl::transformPointCloud(*_reloc_source, *source_tran, T.inverse().cast<float>());
    mypcl::savePLYFileBinary("source_tran.ply", *source_tran);
    sad::Icp3d::Options options;
    options.max_iteration_ = 200;
    options.min_effective_pts_ = 20;
    options.eps_ = 1e-4;
    sad::Icp3d icp(options);
    icp.SetSource(source_tran);
    icp.SetTarget(target);
    SE3 pose;
    Eigen::Matrix4d T_init;
    T_init.setIdentity();
    Eigen::Quaterniond q_init(T_init.block<3, 3>(0, 0));
    Eigen::Vector3d t_init(T_init.block<3, 1>(0, 3));
    pose = SE3(Quatd(q_init.w(), q_init.x(), q_init.y(), q_init.z()), Vec3d(t_init.x(), t_init.y(), t_init.z()));
    bool success;
    success = icp.AlignP2Line(pose);
    Eigen::Matrix4f result_loop = pose.matrix().cast<float>();
    auto result = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
    mypcl::transformPointCloud(*source_tran, *result, result_loop.cast<float>());
    mypcl::savePLYFileBinary("result.ply", *result);
}