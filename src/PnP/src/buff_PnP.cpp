#include <buff/buff_PnP.h>

/**
 * @brief 将旋转矩阵转化为欧拉角
 * @param R 旋转矩阵
 * @return 欧拉角
*/
Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2( R(2,1), R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2( R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return {z, y, x};
}

CoordSolver::CoordSolver()
{
}
CoordSolver::~CoordSolver()
{
}
/*
void CoordSolver::loadParam(std::string coord_path, std::string param_name)
{
    YAML::Node config = YAML::LoadFile(coord_path);

    Eigen::MatrixXd mat_intrinsic(3, 3);
    Eigen::MatrixXd mat_ic(4, 4);
    Eigen::MatrixXd mat_ci(4, 4);
    Eigen::MatrixXd mat_coeff(1, 5);
    Eigen::MatrixXd mat_xyz_offset(1,3);
    Eigen::MatrixXd mat_t_iw(1,3);
    Eigen::MatrixXd mat_angle_offset(1,2);
    
    //初始化弹道补偿参数
    max_iter = config[param_name]["max_iter"].as<int>();
    stop_error = config[param_name]["stop_error"].as<float>();
    R_K_iter = config[param_name]["R_K_iter"].as<int>();

    //初始化内参矩阵
    auto read_vector = config[param_name]["Intrinsic"].as<vector<float>>();
    initMatrix(mat_intrinsic,read_vector);
    eigen2cv(mat_intrinsic,intrinsic);

    //初始化畸变矩阵
    read_vector = config[param_name]["Coeff"].as<vector<float>>();
    initMatrix(mat_coeff,read_vector);
    eigen2cv(mat_coeff,dis_coeff);

    read_vector = config[param_name]["T_iw"].as<vector<float>>();
    initMatrix(mat_t_iw,read_vector);
    t_iw = mat_t_iw.transpose();

    read_vector = config[param_name]["xyz_offset"].as<vector<float>>();
    initMatrix(mat_xyz_offset,read_vector);
    xyz_offset = mat_xyz_offset.transpose();

    read_vector = config[param_name]["angle_offset"].as<vector<float>>();
    initMatrix(mat_angle_offset,read_vector);
    angle_offset = mat_angle_offset.transpose();

    read_vector = config[param_name]["T_ic"].as<vector<float>>();
    initMatrix(mat_ic,read_vector);
    transform_ic = mat_ic;

    read_vector = config[param_name]["T_ci"].as<vector<float>>();
    initMatrix(mat_ci,read_vector);
    transform_ci = mat_ci;

    return ;
}*/

/**
 * @brief 相机坐标系至世界坐标系
 * @param point_camera 相机坐标系下坐标
 * @param rmat 由陀螺仪四元数解算出的旋转矩阵
 * @return 世界坐标系下坐标
 * **/
Eigen::Vector3d CoordSolver::camToWorld(const Eigen::Vector3d &point_camera, const Eigen::Matrix3d &rmat)
{
    //升高维度
    Eigen::Vector4d point_camera_tmp;
    Eigen::Vector4d point_imu_tmp;
    Eigen::Vector3d point_imu;
    Eigen::Vector3d point_world;

    point_camera_tmp << point_camera[0], point_camera[1], point_camera[2], 1;
    point_imu_tmp = transform_ic * point_camera_tmp;
    point_imu << point_imu_tmp[0], point_imu_tmp[1], point_imu_tmp[2];
    point_imu -= t_iw;
    return rmat * point_imu;
}

/**
 * @brief PnP解算目标距离与位姿
 * 
 * @return buff_PnP
 */
void CoordSolver::PnP(const std::vector<cv::Point2f> points_pic)
{
    // Eigen::Matrix3d rmat_imu = src.quat.toRotationMatrix(); // USING_IMU
    Eigen::Matrix3d rmat_imu = Eigen::Matrix3d::Identity(); // NOT_USING_IMU

    // method:PnP解算方法
    int method = cv::SOLVEPNP_ITERATIVE;
    std::vector<cv::Point3d> points_world;

    points_world = {
    {-0.1125,0.027 ,0},
    {-0.1125,-0.027,0},
    {0      ,-0.7  ,-0.05},
    {0.1125 ,-0.027,0},
    {0.1125 ,0.027 ,0}};
    // points_world = {
    // {-0.1125,0.027,0},
    // {-0.1125,-0.027,0},
    // {0,-0.565,-0.05},
    // {0.1125,-0.027,0},
    // {0.1125,0.027,0}};

    cv::Mat rvec;
    cv::Mat rmat;
    cv::Mat tvec;
    Eigen::Matrix3d rmat_eigen;
    Eigen::Vector3d R_center_world = {0,-0.7,-0.05};
    Eigen::Vector3d tvec_eigen;
    Eigen::Vector3d coord_camera;

    cv::solvePnP(points_world, points_pic, intrinsic, dis_coeff, rvec, tvec, false, method);

    PnPInfo result;
    //Pc = R * Pw + T
    Rodrigues(rvec,rmat);
    cv2eigen(rmat, rmat_eigen);
    cv2eigen(tvec, tvec_eigen);

    result.armor_cam = tvec_eigen;
    result.armor_world = camToWorld(result.armor_cam, rmat_imu);
    result.R_cam = (rmat_eigen * R_center_world) + tvec_eigen;
    result.R_world = camToWorld(result.R_cam, rmat_imu);
    // result.euler = rotationMatrixToEulerAngles(transform_ci.block(0,0,2,2) * rmat_imu * rmat_eigen);
    Eigen::Matrix3d rmat_eigen_world = rmat_imu * (transform_ic.block(0, 0, 3, 3) * rmat_eigen);
    // result.euler = rotationMatrixToEulerAngles(rmat_eigen_world);
    result.euler = rotationMatrixToEulerAngles(rmat_eigen_world);
    result.rmat = rmat_eigen_world;

    return ;
}

void callback(const rm_msgs::buff_infer_PnP &Imsg)
{
    std::vector<cv::Point2f> points;
    if(Imsg.Points.size() >= 5)
        for(int i = 0; i < 5; i++){
            points[i].x = Imsg.Points[i * 2];
            points[i].y = Imsg.Points[i * 2 + 1];
        }
    CoordSolver pnp;
    pnp.PnP(points);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<rm_msgs::buff_infer_PnP>("PnP", 10);

    return ;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "buff_PnP"); // 初始化ROS节点
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("infer_PnP", 10, callback);
    ros::spin();
    return 0;
}