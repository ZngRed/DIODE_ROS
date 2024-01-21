#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rm_msgs/A_infer_armor.h>
#include <rm_msgs/A_infer_track.h>
#include <rm_msgs/A_track_predict.h>
#include <rm_msgs/A_update.h>
#include <rm_msgs/A_failed.h>
#include <rm_msgs/Can_send.h>
#include <rm_msgs/Can_receive.h>
#include <tools/coordsolver.h>

#include <iostream>
#include <future>
#include <vector>
#include <fstream>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fmt/format.h>
#include <fmt/color.h>

using namespace cv;
using namespace std;

enum SpinHeading {UNKNOWN, CLOCKWISE, COUNTER_CLOCKWISE};
enum Color {BLUE,RED};

int anti_spin_judge_high_thres = 2e4;   //大于该阈值认为该车已开启陀螺
int anti_spin_judge_low_thres = 2e3;    //小于该阈值认为该车已关闭陀螺
int anti_spin_max_r_multiple = 4.5;
const int max_delta_t = 500;                //使用同一预测器的最大时间间隔(ms)
const int armor_type_wh_thres = 2.8;      //大小装甲板长宽比阈值
const double armor_roi_expand_ratio_width = 1;
const double armor_roi_expand_ratio_height = 2;
const double armor_conf_high_thres = 0.82;  //置信度大于该值的装甲板直接采用
const int max_dead_buffer = 2;              //允许因击打暂时熄灭的装甲板的出现次数
const double max_delta_dist = 0.3;          //两次预测间最大速度(m/s)
const int hero_danger_zone = 99;       //英雄危险距离阈值，检测到有小于该距离的英雄直接开始攻击
const int max_armors = 8;

struct Armor
{
    int id;
    int color;
    int area;
    double conf;
    string key;
    Point2f apex2d[4];
    Rect rect;
    RotatedRect rrect;
    Rect roi;
    Point2f center2d;
    Eigen::Vector3d center3d_cam;
    Eigen::Vector3d center3d_world;
    Eigen::Vector3d euler;
    Eigen::Vector3d predict;

    TargetType type;
};

class ArmorTracker
{
public:
    Armor prev_armor;                       //上一次装甲板
    Armor last_armor;                       //本次装甲板
    bool is_initialized;                    //是否完成初始化
    int last_selected_timestamp;            //该Tracker上次被选为目标tracker时间戳
    int prev_timestamp;                     //上次装甲板时间戳
    int last_timestamp;                     //本次装甲板时间戳
    int history_type_sum;                   //历史次数之和
    int selected_cnt;                       //该Tracker被选为目标tracker次数和
    const int max_history_len = 4;          //历史信息队列最大长度
    double hit_score;                       //该tracker可能作为目标的分数,由装甲板旋转角度,距离,面积大小决定
    double velocity;
    double radius;
    string key;

    std::deque<Armor> history_info;//目标队列

    ArmorTracker(Armor src,int src_timestamp);
    bool update(Armor new_armor, int new_timestamp);
    bool calcTargetScore();
};