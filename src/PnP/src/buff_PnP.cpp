#include <buff/buff_PnP.h>

/**
 * @brief PnP解算目标距离与位姿
 * 
 * @param points_pic 目标点,长度为5为大符模式
 * 
 * @return buff_PnP
 */
void PnP(const std::vector<Point2f> &points_pic)
{
    // Eigen::Matrix3d rmat_imu = src.quat.toRotationMatrix(); // USING_IMU
    Eigen::Matrix3d rmat_imu = Eigen::Matrix3d::Identity(); // NOT_USING_IMU

    // method:PnP解算方法
    int method = SOLVEPNP_ITERATIVE;
    std::vector<Point3d> points_world;

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

    Mat rvec;
    Mat rmat;
    Mat tvec;
    Eigen::Matrix3d rmat_eigen;
    Eigen::Vector3d R_center_world = {0,-0.7,-0.05};
    Eigen::Vector3d tvec_eigen;
    Eigen::Vector3d coord_camera;

    solvePnP(points_world, points_pic, intrinsic, dis_coeff, rvec, tvec, false, method);

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

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "buff_PnP"); // 初始化ROS节点
    ros::NodeHandle nh;
    ros::ImageTransport it(nh);
    ros::Subscriber image_sub = it.subscribe("buff_PnP", 10, PnP);
    ros::spin();
    return 0;
}