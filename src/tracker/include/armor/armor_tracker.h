#include <ros/ros.h>
#include <rm_msgs/A_infer_armor.h>
#include <rm_msgs/A_infer_track.h>
#include <rm_msgs/A_track_predict.h>
#include <rm_msgs/A_update.h>
#include <tools/coordsolver.h>

#include <iostream>
#include <future>
#include <vector>
#include <fstream>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

using namespace cv;
using namespace std;

// const string camera_param_path = "/home/dhu/Model/coord_param.yaml";
// const string camera_name = "KE0200110075";     // 步兵（4号)

enum SpinHeading {UNKNOWN, CLOCKWISE, COUNTER_CLOCKWISE};
enum Color {BLUE,RED};

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