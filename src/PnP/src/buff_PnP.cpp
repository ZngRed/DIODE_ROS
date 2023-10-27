#include <buff_PnP/buff_PnP.h>

void PnP()
{
    std::vector<Point3d> points_world;

    points_world = {
    {-0.1125,0.027,0},
    {-0.1125,-0.027,0},
    {0,-0.7,-0.05},
    {0.1125,-0.027,0},
    {0.1125,0.027,0}};
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
    return 0;
}