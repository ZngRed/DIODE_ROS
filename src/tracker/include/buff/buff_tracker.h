#include <ros/ros.h>
#include <rm_msgs/B_infer_fan.h>
#include <rm_msgs/B_infer_track.h>
#include <tools/coordsolver.h>

#include <iostream>

#include <future>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

// #include "../general/general.h"

using namespace cv;
using namespace std;

struct Fan
{
    int id;
    int color;
    double conf;
    string key;
    Point2f apex2d[5];

    Eigen::Vector3d centerR3d_cam;
    Eigen::Vector3d centerR3d_world;
    Eigen::Vector3d armor3d_cam;
    Eigen::Vector3d armor3d_world;
    Eigen::Vector3d euler;
    Eigen::Matrix3d rmat;
};

class FanTracker
{
public:
    Fan prev_fan;                           // 上一次装甲板
    Fan last_fan;                           // 本次装甲板
    bool is_last_fan_exists;                // 是否存在上一次扇叶
    bool is_initialized;                    // 是否完成初始化
    double rotate_speed;                    // 角速度
    int max_history_len = 2;                // 队列长度
    int prev_timestamp;                     // 上次装甲板时间戳
    int last_timestamp;                     // 本次装甲板时间戳

    std::deque<Fan> history_info; // 目标队列

    FanTracker(Fan src,int src_timestamp);
    bool update(Fan new_fan, int new_timestamp);
// // class buff
// public:
//     const int max_lost_cnt = 4;//最大丢失目标帧数
//     const int max_v = 4;       //最大旋转速度(rad/s)
//     const int max_delta_t = 100; //使用同一预测器的最大时间间隔(ms)
//     const double fan_length = 0.7; //大符臂长(R字中心至装甲板中心)
//     const double no_crop_thres = 2e-3;      //禁用ROI裁剪的装甲板占图像面积最大面积比值

//     bool is_last_target_exists;
//     int lost_cnt;
//     // int last_timestamp;
//     double last_target_area;
//     double last_bullet_speed;
//     Point2i last_roi_center;
//     Point2i roi_offset;
//     Size2d input_size;
};