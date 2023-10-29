#include <iostream>

#include <ctime>
#include <future>
#include <random>
#include <vector>

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <matplotlibcpp.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace cv;
using namespace plt;

class buff_predict
{
private:
    const double fan_length = 0.7
    evalRMSE(double params[4]);
    calcAimingAngleOffset(double params[4], double t0, double t1 , int mode);
    predict(double speed, double dist, int timestamp, double &result);
}