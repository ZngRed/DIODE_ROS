#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rm_msgs/B_infer_fan.h>
#include <rm_msgs/B_infer_track.h>

#include <inference_engine.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include <unistd.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <fmt/format.h>
#include <fmt/color.h>
#include <glog/logging.h>

using namespace InferenceEngine;
using namespace std;
using namespace cv;

struct BuffObject
{
    Point2f apex[5];
    cv::Rect_<float> rect;
    int cls;
    int color;
    float prob;
    std::vector<cv::Point2f> pts;
};

struct GridAndStride
{
    int grid0;
    int grid1;
    int stride;
};

class buff_infer
{
private:
    InferenceEngine::Core ie;
    InferenceEngine::CNNNetwork network;
    InferenceEngine::ExecutableNetwork executable_network;
    InferenceEngine::InferRequest infer_request;
    MemoryBlob::CPtr moutput;
    std::string input_name;
    std::string output_name;
    Eigen::Matrix<float,3,3> transfrom_matrix;
public:
    buff_infer();
    ~buff_infer();

    void model_init();
    void infer(Mat &img, std::vector<Point2f> &points);
// class buff
public:
    const int max_lost_cnt = 4;//最大丢失目标帧数
    const int max_v = 4;       //最大旋转速度(rad/s)
    const int max_delta_t = 100; //使用同一预测器的最大时间间隔(ms)
    const double fan_length = 0.7; //大符臂长(R字中心至装甲板中心)
    const double no_crop_thres = 2e-3;      //禁用ROI裁剪的装甲板占图像面积最大面积比值

    bool is_last_target_exists;
    int lost_cnt;
    int last_timestamp;
    double last_target_area;
    double last_bullet_speed;
    Point2i last_roi_center;
    Point2i roi_offset;
    Size2d input_size;

    Point2i cropImageByROI(Mat &img);
};