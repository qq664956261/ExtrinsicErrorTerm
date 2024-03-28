#include <ExtrinsicErrorTerm/ExtrinsicErrorTerm.h>
ExtrinsicErrorTerm::ExtrinsicErrorTerm()
{
    _leftFrontCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    _leftBackCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    _CloudAll.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    _kdtreeFromLeftBack.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    _kdtreeFromLeftFront.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
}
ExtrinsicErrorTerm::~ExtrinsicErrorTerm()
{
}

int ExtrinsicErrorTerm::readPose(const std::string filepath)
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

int ExtrinsicErrorTerm::readSonarWaveData(const std::string filepath)
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

int ExtrinsicErrorTerm::buildMap()
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

int ExtrinsicErrorTerm::timeStampSynchronization(double sonarWaveTimeStamp)
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

int ExtrinsicErrorTerm::Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
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
int ExtrinsicErrorTerm::Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Matrix3d R12, Eigen::Vector3d t12)
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
        Eigen::Matrix4d T_back_base;
        T_back_base.setIdentity();
        Eigen::Quaterniond q_back_base = Eigen::AngleAxisd(_leftBackYaw, Eigen::Vector3d::UnitZ()) *
                                         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        q_back_base.normalize();
        T_back_base.block<3, 3>(0, 0) = q_back_base.toRotationMatrix();
        T_back_base.block<3, 1>(0, 3) = Eigen::Vector3d(_leftBackX, _leftBackY, 0);
        Eigen::Matrix4d T_base_back = T_back_base.inverse();
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
        Eigen::Matrix4d T_front_base;
        T_front_base.setIdentity();
        Eigen::Quaterniond q_front_base = Eigen::AngleAxisd(_leftFrontyaw, Eigen::Vector3d::UnitZ()) *
                                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        T_front_base.block<3, 3>(0, 0) = q_front_base.toRotationMatrix();
        T_front_base.block<3, 1>(0, 3) = Eigen::Vector3d(_leftFrontX, _leftFronty, 0);
        Eigen::Matrix4d T_base_front = T_front_base.inverse();
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
        Eigen::Matrix4d T_back_base = T_base_back.inverse();
        Eigen::Vector3d ypr;
        Eigen::Matrix3d R_back_base = T_back_base.block<3, 3>(0, 0);
        ypr = R_back_base.eulerAngles(2, 1, 0);
        Eigen::Vector3d t_back_base = T_back_base.block<3, 1>(0, 3);
        // std::cout<<"R_back_base:"<<ypr<<std::endl;
        // std::cout<<"t_back_base:"<<t_back_base<<std::endl;
        // std::cout<<"R12:"<<R12<<std::endl;
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

void ExtrinsicErrorTerm::align()
{
    // pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_out;
    // voxel_grid_out.setLeafSize(0.05, 0.05, 0.05);
    // voxel_grid_out.setInputCloud(_leftBackCloud);
    // voxel_grid_out.filter(*_leftBackCloud);
    // voxel_grid_out.setInputCloud(_leftFrontCloud);
    // voxel_grid_out.filter(*_leftFrontCloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputCloud(_leftBackCloud);
    icp.setInputTarget(_leftFrontCloud);
    icp.setMaxCorrespondenceDistance(0.04);
    icp.setMaximumIterations(1000);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(0.1);
    icp.align(*output);
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    std::cout << "transformation:" << transformation << std::endl;
    for (int i = 0; i < _leftFrontCloud->points.size(); i++)
    {
        pcl::PointXYZRGB p;
        p.x = _leftFrontCloud->points[i].x;
        p.y = _leftFrontCloud->points[i].y;
        p.z = _leftFrontCloud->points[i].z;
        p.r = 255;
        p.g = 0;
        p.b = 0;
        result->points.push_back(p);
    }
    for (int i = 0; i < output->points.size(); i++)
    {
        pcl::PointXYZRGB p;
        p.x = output->points[i].x;
        p.y = output->points[i].y;
        p.z = output->points[i].z;
        p.r = 0;
        p.g = 255;
        p.b = 0;
        result->points.push_back(p);
    }
    result->height = 1;
    result->width = result->points.size();
    if (result->points.size() != 0)
        pcl::io::savePCDFileASCII("result.pcd", *result);
}

void ExtrinsicErrorTerm::ceresAlign()
{

    Eigen::Vector3d temp_t;
    temp_t.setZero();
    Eigen::Quaterniond temp_q;
    temp_q.setIdentity();
    Eigen::Matrix4d T_back_base_temp;
    T_back_base_temp.setIdentity();
    Eigen::Matrix4d T_front_base_temp;
    T_front_base_temp.setIdentity();
    Eigen::Quaterniond q_back_base_temp = Eigen::AngleAxisd(_leftBackYaw, Eigen::Vector3d::UnitZ()) *
                                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    q_back_base_temp.normalize();
    T_back_base_temp.block<3, 3>(0, 0) = q_back_base_temp.toRotationMatrix();
    T_back_base_temp.block<3, 1>(0, 3) = Eigen::Vector3d(_leftBackX, _leftBackY, 0);

    Eigen::Quaterniond q_front_base_temp = Eigen::AngleAxisd(_leftFrontyaw, Eigen::Vector3d::UnitZ()) *
                                           Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                           Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    q_front_base_temp.normalize();
    T_front_base_temp.block<3, 3>(0, 0) = q_front_base_temp.toRotationMatrix();
    T_front_base_temp.block<3, 1>(0, 3) = Eigen::Vector3d(_leftFrontX, _leftFronty, 0);

    if (_leftBackBase)
    {
        Eigen::Matrix4d T_back_front;
        T_back_front = T_back_base_temp * T_front_base_temp.inverse();
        Eigen::Quaterniond q_back_front(T_back_front.block<3, 3>(0, 0));
        temp_q = q_back_front;
        temp_t = T_back_front.block<3, 1>(0, 3);
        std::cout << "T_back_front.block<3, 3>(0, 0):" << T_back_front.block<3, 3>(0, 0) << std::endl;
    }
    else
    {
        Eigen::Matrix4d T_front_back;
        T_front_back = T_front_base_temp * T_back_base_temp.inverse();
        Eigen::Quaterniond q_front_back(T_front_back.block<3, 3>(0, 0));
        temp_q = q_front_back;
        temp_t = T_front_back.block<3, 1>(0, 3);
        std::cout << "T_front_back.block<3, 3>(0, 0):" << T_front_back.block<3, 3>(0, 0) << std::endl;
    }
    double para_t[3];
    double para_q[4];
    double se3[6];
    para_t[0] = temp_t.x();
    para_t[1] = temp_t.y();
    para_t[2] = temp_t.z();

    para_q[0] = temp_q.x();
    para_q[1] = temp_q.y();
    para_q[2] = temp_q.z();
    para_q[3] = temp_q.w();
    Sophus::SE3 SE3_T(temp_q, temp_t);
    Eigen::Matrix<double, 6, 1> se3_T = SE3_T.log();
    se3[0] = se3_T[0];
    se3[1] = se3_T[1];
    se3[2] = se3_T[2];
    se3[3] = se3_T[3];
    se3[4] = se3_T[4];
    se3[5] = se3_T[5];
    std::cout << "se3_T:" << se3_T << std::endl;
    std::cout << "para_q[0]:" << para_q[0] << std::endl;
    std::cout << "para_q[1]:" << para_q[1] << std::endl;
    std::cout << "para_q[2]:" << para_q[2] << std::endl;
    std::cout << "para_q[3]:" << para_q[3] << std::endl;
    std::cout << "para_t[0]:" << para_t[0] << std::endl;
    std::cout << "para_t[1]:" << para_t[1] << std::endl;
    std::cout << "para_t[2]:" << para_t[2] << std::endl;

    for (int i = 0; i < _SonarWaveDatas.size(); i++)
    {
        std::vector<double> data = _SonarWaveDatas[i];
        double sonarTimeStamp = data[0] * 1e-6;
        int index = timeStampSynchronization(sonarTimeStamp);
        if (_leftBackBase)
            Sonar2cloud(SonarIndex::left_back, i, index, _leftBackCloud);
        else
            Sonar2cloud(SonarIndex::left_front, i, index, _leftFrontCloud);
    }
    if (_leftBackBase)
        _kdtreeFromLeftBack->setInputCloud(_leftBackCloud);
    else
        _kdtreeFromLeftFront->setInputCloud(_leftFrontCloud);
    for (int num = 0; num < 1; num++)
    {
        ceres::Problem problem;
        ceres::LocalParameterization *local_parameterization = new ceres::EigenQuaternionParameterization();
        ceres::LocalParameterization *local_param = new SE3Param();
        if (_useAutoDiff)
        {
            problem.AddParameterBlock(para_q, 4, local_parameterization);
            problem.AddParameterBlock(para_t, 3);
            problem.SetParameterLowerBound(para_t, 0, 0.29);
            problem.SetParameterUpperBound(para_t, 0, 0.3);
            problem.SetParameterLowerBound(para_t, 1, -0.04);
            problem.SetParameterUpperBound(para_t, 1, -0.03);
            problem.SetParameterLowerBound(para_q, 0, -0.0001);
            problem.SetParameterUpperBound(para_q, 0, 0.0001);
            problem.SetParameterLowerBound(para_q, 1, -0.0001);
            problem.SetParameterUpperBound(para_q, 1, 0.0001);
            problem.SetParameterLowerBound(para_q, 2, -0.07);
            problem.SetParameterUpperBound(para_q, 2, -0.06);
            problem.SetParameterLowerBound(para_q, 3, 0.999);
            problem.SetParameterUpperBound(para_q, 3, 1);
        }
        else
        {
            problem.AddParameterBlock(se3, 6, local_param);
        }
        // problem.SetParameterBlockConstant(para_q);
        // problem.SetParameterBlockConstant(para_t);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        for (int i = 0; i < _SonarWaveDatas.size(); i++)
        {
            std::vector<double> data = _SonarWaveDatas[i];
            double sonarTimeStamp = data[0] * 1e-6;
            int index = timeStampSynchronization(sonarTimeStamp);

            float fov_rad = 0.0, sonar_base_x = 0.0, sonar_base_y = 0.0, sonar_base_yaw = 0.0;
            float length;
            if (_leftBackBase)
            {
                fov_rad = _leftFrontFovRad;
                sonar_base_x = _leftFrontX;
                sonar_base_y = _leftFronty;
                sonar_base_yaw = _leftFrontyaw;
                length = _SonarWaveDatas[i][1];
            }
            else
            {
                fov_rad = _leftBackFovRad;
                sonar_base_x = _leftBackX;
                sonar_base_y = _leftBackY;
                sonar_base_yaw = _leftBackYaw;
                length = _SonarWaveDatas[i][2];
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
            Eigen::Quaterniond cur_Q_ = Eigen::AngleAxisd(_Poses[index][6], Eigen::Vector3d::UnitZ()) *
                                        Eigen::AngleAxisd(_Poses[index][5], Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(_Poses[index][4], Eigen::Vector3d::UnitX());
            cur_Q_.normalize();
            T_wc.block<3, 3>(0, 0) = cur_Q_.toRotationMatrix();
            T_wc.block<3, 1>(0, 3) = Eigen::Vector3d(_Poses[index][1], _Poses[index][2], _Poses[index][3]);
            Eigen::Vector4d p_ = Eigen::Vector4d(base_x, base_y, 0, 1);
            Eigen::Vector4d p_w = T_wc * p_;
            Eigen::Vector3d p_w_3 = Eigen::Vector3d(p_w[0], p_w[1], p_w[2]);
            pcl::PointXYZI pointSel;
            pointSel.x = p_w_3[0];
            pointSel.y = p_w_3[1];
            pointSel.z = p_w_3[2];

            if (_leftBackBase)
            {
                _kdtreeFromLeftBack->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            }
            else
            {
                _kdtreeFromLeftFront->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            }

            // if (pointSearchSqDis[0] > 0.01)
            // {
            //     std::cout << "pointSearchSqDis[0]:" << pointSearchSqDis[0] << std::endl;
            //     continue;
            // }
            if (_leftBackBase)
            {
                ceres::LossFunction *loss_function = new ceres::HuberLoss(10);
                Eigen::Matrix4d T_back_base;
                T_back_base.setIdentity();
                Eigen::Quaterniond q_back_base = Eigen::AngleAxisd(_leftBackYaw, Eigen::Vector3d::UnitZ()) *
                                                 Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                                 Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
                q_back_base.normalize();
                T_back_base.block<3, 3>(0, 0) = q_back_base.toRotationMatrix();
                T_back_base.block<3, 1>(0, 3) = Eigen::Vector3d(_leftBackX, _leftBackY, 0);
                Eigen::Matrix4d T_base_back = T_back_base.inverse();
                Eigen::Matrix4d T_w_base = T_wc;
                Eigen::Matrix4d T_w_back = T_w_base * T_base_back;
                Eigen::Quaterniond q_w_back(T_w_back.block<3, 3>(0, 0));
                Eigen::Vector3d t_w_back(T_w_back.block<3, 1>(0, 3));
                ceres::CostFunction *cost_function = sonarEdgeFactor::Create(
                    Eigen::Vector3d(sonar_x, sonar_y, sonar_z),
                    Eigen::Vector3d(_leftBackCloud->points[pointSearchInd[0]].x, _leftBackCloud->points[pointSearchInd[0]].y, _leftBackCloud->points[pointSearchInd[0]].z),
                    q_w_back,
                    t_w_back);
                problem.AddResidualBlock(cost_function, NULL, para_q, para_t);
            }
            else
            {
                Eigen::Matrix4d T_front_base;
                T_front_base.setIdentity();
                Eigen::Quaterniond q_front_base = Eigen::AngleAxisd(_leftFrontyaw, Eigen::Vector3d::UnitZ()) *
                                                  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                                  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
                T_front_base.block<3, 3>(0, 0) = q_front_base.toRotationMatrix();
                T_front_base.block<3, 1>(0, 3) = Eigen::Vector3d(_leftFrontX, _leftFronty, 0);
                Eigen::Matrix4d T_base_front = T_front_base.inverse();
                Eigen::Matrix4d T_w_base = T_wc;
                Eigen::Matrix4d T_w_front = T_w_base * T_base_front;
                Eigen::Quaterniond q_w_front(T_w_front.block<3, 3>(0, 0));
                Eigen::Vector3d t_w_front(T_w_front.block<3, 1>(0, 3));
                if (_useAutoDiff)
                {
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(10);

                    ceres::CostFunction *cost_function = sonarEdgeFactor::Create(
                        Eigen::Vector3d(sonar_x, sonar_y, sonar_z),
                        Eigen::Vector3d(_leftFrontCloud->points[pointSearchInd[0]].x, _leftFrontCloud->points[pointSearchInd[0]].y, _leftFrontCloud->points[pointSearchInd[0]].z),
                        q_w_front,
                        t_w_front);
                    problem.AddResidualBlock(cost_function, NULL, para_q, para_t);
                }
                else
                {
                    sonarFactor *cost_function = new sonarFactor(Eigen::Vector3d(sonar_x, sonar_y, sonar_z),
                                                                 Eigen::Vector3d(_leftFrontCloud->points[pointSearchInd[0]].x, _leftFrontCloud->points[pointSearchInd[0]].y, _leftFrontCloud->points[pointSearchInd[0]].z),
                                                                 q_w_front,
                                                                 t_w_front);
                    problem.AddResidualBlock(cost_function, NULL, se3);
                }
            }
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.num_threads = 8;
        options.minimizer_progress_to_stdout = true;
        options.max_solver_time_in_seconds = 600;
        options.max_num_iterations = 1000;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 1)
        {
            std::cout << " converge:" << summary.final_cost << std::endl;
        }
        else
        {
            std::cout << " not converge :" << summary.final_cost << std::endl;
        }
    }
    Eigen::Quaterniond q_result;
    Eigen::Matrix3d R_result;
    Eigen::Vector3d t_result;
    if (!_useAutoDiff)
    {
        Eigen::Matrix<double, 6, 1> vresult;
        vresult << se3[0], se3[1], se3[2], se3[3], se3[4], se3[5];
        Sophus::SE3 T_result = Sophus::SE3::exp(vresult);
        Eigen::Matrix<double, 7, 1> pose_result;
        pose_result.block<3, 1>(0, 0) = T_result.translation();
        pose_result.block<4, 1>(3, 0) = T_result.unit_quaternion().coeffs();
        std::cout << "pose_result:" << pose_result << std::endl;

        Eigen::Quaterniond q_result_temp(pose_result[6], pose_result[3], pose_result[4], pose_result[5]);
        R_result = q_result_temp.toRotationMatrix();
        t_result = T_result.translation();
    }
    else
    {
        Eigen::Quaterniond q_result_temp(para_q[3], para_q[0], para_q[1], para_q[2]);
        R_result = q_result_temp.toRotationMatrix();
        Eigen::Vector3d t_result_temp(para_t[0], para_t[1], para_t[2]);
        t_result = t_result_temp;
    }
    Eigen::Vector3d ypr;
    ypr = R_result.eulerAngles(2, 1, 0);
    std::cout << "R_result:" << R_result << std::endl;
    std::cout << "ypr:" << ypr << std::endl;
    std::cout << "t_result:" << t_result << std::endl;
    std::cout << "para_q[0]:" << para_q[0] << std::endl;
    std::cout << "para_q[1]:" << para_q[1] << std::endl;
    std::cout << "para_q[2]:" << para_q[2] << std::endl;
    std::cout << "para_q[3]:" << para_q[3] << std::endl;
    Eigen::AngleAxisd rotation_vector(R_result);
    for (int i = 0; i < _SonarWaveDatas.size(); i++)
    {
        std::vector<double> data = _SonarWaveDatas[i];
        double sonarTimeStamp = data[0] * 1e-6;
        int index = timeStampSynchronization(sonarTimeStamp);
        if (_leftBackBase)
        {
            Sonar2cloud(SonarIndex::left_front, i, index, _leftFrontCloud, R_result, t_result);
        }
        else
        {
            Sonar2cloud(SonarIndex::left_back, i, index, _leftBackCloud, R_result, t_result);
        }
    }
    _CloudAll->height = 1;
    _CloudAll->width = _CloudAll->points.size();
    if (_CloudAll->points.size() != 0)
    {
        pcl::io::savePCDFileASCII("CloudAllceres.pcd", *_CloudAll);
        pcl::io::savePLYFileBinary("CloudAllceres.ply", *_CloudAll);
    }
}