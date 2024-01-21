#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rm_msgs/A_infer_armor.h>
#include <rm_msgs/A_infer_track.h>
#include <rm_msgs/A_update.h>
#include <rm_msgs/A_failed.h>
#include <rm_msgs/Can_send.h>

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

enum SpinHeading {UNKNOWN, CLOCKWISE, COUNTER_CLOCKWISE};
enum Color {BLUE,RED};

struct ArmorObject
{
    cv::Point2f apex[4];
    cv::Rect_<float> rect;
    int cls;
    int color;
    int area;
    float prob;
    std::vector<cv::Point2f> pts;
};

struct GridAndStride
{
    int grid0;
    int grid1;
    int stride;
};

class armor_infer
{
private:
    InferenceEngine::Core ie;
    InferenceEngine::CNNNetwork network; // 网络
    InferenceEngine::ExecutableNetwork executable_network; // 可执行网络
    InferenceEngine::InferRequest infer_request; // 推理请求
    MemoryBlob::CPtr moutput;
    std::string input_name;
    std::string output_name;
    Eigen::Matrix<float,3,3> transfrom_matrix;
public:
    armor_infer();
    ~armor_infer();
    
    void model_init();
    bool infer(Mat &img);

};