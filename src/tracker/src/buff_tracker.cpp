#include <buff/buff_tracker.h>

// class buff
const int max_v = 4;       //最大旋转速度(rad/s)
const int max_delta_t = 10000; //使用同一预测器的最大时间间隔(ms)
bool is_last_target_exists;
int lost_cnt;
int src_timestamp;

int mode = 4;

Eigen::Matrix3d rmat_imu = Eigen::Matrix3d::Identity();
Fan last_fan;
std::vector<Fan> fans;
std::vector<FanTracker> trackers;
CoordSolver coordsolver;
ros::Publisher pub;

bool chooseTarget(vector<Fan> &fans, Fan &target)
{
    float max_area = 0;
    int target_idx = 0;
    int target_fan_cnt = 0;
    int other_fan_cnt = 0;
    for (auto fan : fans)
    {
        if (fan.id == 0){
            target = fan;
            target_fan_cnt++;
        }else if (fan.id == 1){
            other_fan_cnt++;
        }
    }
    cout<<"id_0: "<<target_fan_cnt<<" id_1: "<<other_fan_cnt<<endl;
    if (target_fan_cnt >= 1)
        return true;
    else
        return false;
}

/**
 * @brief 构造一个Tracker对象
 * 
 * @param src 对象
 */
FanTracker::FanTracker(Fan src, int src_timestamp)
{
    last_fan = src;
    last_timestamp = src_timestamp;
    is_initialized = false;
    is_last_fan_exists = false;
    history_info.push_back(src);
}

bool FanTracker::update(Fan new_fan,int new_timestamp)
{
    is_last_fan_exists = true;
    if (history_info.size() < max_history_len)
    {
        history_info.push_back(new_fan);
    }
    else
    {
        is_initialized = true;
        history_info.pop_front();
        history_info.push_back(new_fan);
    }

    prev_fan = last_fan;
    prev_timestamp = last_timestamp;
    
    last_fan = new_fan;
    last_timestamp = new_timestamp;

    return true;
}

void callback_track(const rm_msgs::B_infer_track::ConstPtr& Imsg)
{
    src_timestamp = Imsg->src_timestamp;
    std::cout << "track subscribed! " << src_timestamp << endl;
    if (trackers.size() != 0){
        //维护Tracker队列，删除过旧的Tracker
        for (auto iter = trackers.begin(); iter != trackers.end();){
            //删除元素后迭代器会失效，需先行获取下一元素
            auto next = iter;
            // cout<<(*iter).second.last_timestamp<<"  "<<Imsg->src_timestamp<<endl;
            if ((Imsg->src_timestamp - (*iter).last_timestamp) > max_delta_t)
                next = trackers.erase(iter);
            else
                ++next;
            iter = next;
        }
    }
    std::vector<FanTracker> trackers_tmp;
    for (auto fan = fans.begin(); fan != fans.end(); ++fan){
        if (trackers.size() == 0){
            FanTracker fan_tracker((*fan), Imsg->src_timestamp);
            trackers_tmp.push_back(fan_tracker);
        }else{
            //1e9无实际意义，仅用于以非零初始化
            double min_v = 1e9;
            int min_last_delta_t = 1e9;
            bool is_best_candidate_exist = false;
            std::vector<FanTracker>::iterator best_candidate;
            for (auto iter = trackers.begin(); iter != trackers.end(); iter++){
                double delta_t;
                Eigen::AngleAxisd angle_axisd;
                double rotate_speed;
                double sign;
                //----------------------------计算角度,求解转速----------------------------
                //若该扇叶完成初始化,且隔一帧时间较短
                if ((*iter).is_initialized && (Imsg->src_timestamp - (*iter).prev_timestamp) < max_delta_t){
                    delta_t = Imsg->src_timestamp - (*iter).prev_timestamp;
                    //目前扇叶到上一次扇叶的旋转矩阵
                    auto relative_rmat = (*iter).prev_fan.rmat.transpose() * (*fan).rmat;
                    angle_axisd = Eigen::AngleAxisd(relative_rmat);
                    auto rotate_axis_world = (*iter).last_fan.rmat * angle_axisd.axis();
                    sign = ((*fan).centerR3d_world.dot(rotate_axis_world) > 0 ) ? 1 : -1;
                }else{
                    delta_t = Imsg->src_timestamp - (*iter).last_timestamp;
                    //目前扇叶到上一次扇叶的旋转矩阵
                    auto relative_rmat = (*iter).last_fan.rmat.transpose() * (*fan).rmat;
                    //TODO:使用点乘判断旋转方向
                    angle_axisd = Eigen::AngleAxisd(relative_rmat);
                    auto rotate_axis_world = (*fan).rmat * angle_axisd.axis();
                    sign = ((*fan).centerR3d_world.dot(rotate_axis_world) > 0 ) ? 1 : -1;
                }
                rotate_speed = sign * (angle_axisd.angle()) / delta_t * 1e3;//计算角速度(rad/s)
                if (abs(rotate_speed) <= max_v && abs(rotate_speed) <= min_v && (Imsg->src_timestamp - (*iter).last_timestamp) <= min_last_delta_t){
                    min_last_delta_t = Imsg->src_timestamp - (*iter).last_timestamp;
                    min_v = rotate_speed;
                    best_candidate = iter;
                    is_best_candidate_exist = true;
                }
            }
            if (is_best_candidate_exist){
                (*best_candidate).update((*fan), Imsg->src_timestamp);
                (*best_candidate).rotate_speed = min_v;
            }else{
                FanTracker fan_tracker((*fan), Imsg->src_timestamp);
                trackers_tmp.push_back(fan_tracker);
            }
        }
    }
    for (auto new_tracker : trackers_tmp){
        trackers.push_back(new_tracker);
    }


    ///------------------------检测待激活扇叶是否存在----------------------------
    Fan target;
    bool is_target_exists = chooseTarget(fans, target);

    //若不存在待击打扇叶则返回false
    if (!is_target_exists)
    {
        std::cout<<"target does not exist."<<endl;
        lost_cnt++;
        is_last_target_exists = false;
        // data = {(float)0, (float)0, (float)0, 0, 0, 0, 1};
        // LOG(WARNING) <<"[BUFF] No available target fan or Multiple target fan detected!";
        return ;
    }

    int avail_tracker_cnt = 0;
    double rotate_speed_sum = 0;
    double mean_rotate_speed = 0;
    Eigen::Vector3d r_center_sum = {0, 0, 0};
    Eigen::Vector3d mean_r_center = {0, 0, 0};

    //计算平均转速与平均R字中心坐标
    for(auto tracker: trackers)
    {
        if (tracker.is_last_fan_exists && tracker.last_timestamp == Imsg->src_timestamp)
        {
            rotate_speed_sum += tracker.rotate_speed;
            r_center_sum += tracker.last_fan.centerR3d_world;
            avail_tracker_cnt++;
        }
    }
    //若不存在可用的扇叶则返回false
    if (avail_tracker_cnt == 0)
    {
        std::cout<<"no available target."<<endl;
        // LOG(WARNING) <<"[BUFF] No available fan tracker exist!";
        // data = {(float)0, (float)0, (float)0, 0, 0, 0, 1};
        lost_cnt++;
        return ;
    }
    mean_rotate_speed = rotate_speed_sum / avail_tracker_cnt;
    mean_r_center = r_center_sum / avail_tracker_cnt;
    Eigen::Matrix3d last_fan_rmat_transpose = last_fan.rmat.transpose();
    //pub
    rm_msgs::B_track_predict Omsg;
    Omsg.mode = mode;
    Omsg.mean_rotate_speed = mean_rotate_speed;
    Omsg.mean_r_center_norm = mean_r_center.norm();
    Omsg.src_timestamp = Imsg->src_timestamp;
    Omsg.target_armor3d_world.x = target.armor3d_world[0];
    Omsg.target_armor3d_world.y = target.armor3d_world[1];
    Omsg.target_armor3d_world.z = target.armor3d_world[2];
    Omsg.target_centerR3d_world.x = target.centerR3d_world[0];
    Omsg.target_centerR3d_world.y = target.centerR3d_world[1];
    Omsg.target_centerR3d_world.z = target.centerR3d_world[2];
    for(int i = 0; i < 9; i++){
        Omsg.rmat_imu[i] = rmat_imu(i);
        Omsg.target_rmat[i] = target.rmat(i);
        Omsg.last_fan_rmat_transpose[i] = last_fan_rmat_transpose(i);
    }
    Omsg.is_last_target_exists = true;
    ros::NodeHandle nh;
    std::cout<<"The target is going to be published !"<<endl;
    pub = nh.advertise<rm_msgs::B_track_predict>("B_track_predict", 10);
    pub.publish(Omsg);
    //update
    is_last_target_exists = true;
    last_fan = target;
    fans.clear();
    return ;
}

void callback_fan(const rm_msgs::B_infer_fan::ConstPtr& Imsg)
{
    Fan fan;
    fan.id = Imsg->cls;
    fan.color = Imsg->color;
    fan.conf = Imsg->prob;
    if (Imsg->color == 0)
        fan.key = "B" + string(Imsg->cls == 0 ? "Activated" : "Target");
    if (Imsg->color == 1)
        fan.key = "R" + string(Imsg->cls == 0 ? "Activated" : "Target");
    fan.apex2d[0].x = Imsg->apex_0.x;
    fan.apex2d[0].y = Imsg->apex_0.y;
    fan.apex2d[1].x = Imsg->apex_1.x;
    fan.apex2d[1].y = Imsg->apex_1.y;
    fan.apex2d[2].x = Imsg->apex_2.x;
    fan.apex2d[2].y = Imsg->apex_2.y;
    fan.apex2d[3].x = Imsg->apex_3.x;
    fan.apex2d[3].y = Imsg->apex_3.y;
    fan.apex2d[4].x = Imsg->apex_4.x;
    fan.apex2d[4].y = Imsg->apex_4.y;
    for(int i = 0; i < 5; i++)
        fan.apex2d[i] += Point2f((float)Imsg->roi_offset.x,(float)Imsg->roi_offset.y);
    // std::cout<<"fan subscribed! "<<src_timestamp<<" "<<fan.id<<endl;
    std::vector<Point2f> points_pic(fan.apex2d, fan.apex2d + 5);
    TargetType target_type = BUFF;
    // PNP
    // std::cout<<"fanfanfan"<<endl;
    auto pnp_result = coordsolver.pnp(points_pic, rmat_imu, target_type, SOLVEPNP_ITERATIVE);
    // cout<<"rmat_imu:"<<endl<<rmat_imu<<endl<<"*---------------*"<<endl;
    fan.armor3d_cam = pnp_result.armor_cam;
    fan.armor3d_world = pnp_result.armor_world;
    fan.centerR3d_cam = pnp_result.R_cam;
    fan.centerR3d_world = pnp_result.R_world;
    fan.euler = pnp_result.euler;
    fan.rmat = pnp_result.rmat;

    fans.push_back(fan);
    // std::cout<<"fan init done!"<<endl;
    return ;
}

int main(int argc,char** argv)
{
    coordsolver.loadParam(camera_param_path,camera_name);
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "buff_tracker"); // 初始化ROS节点
    ros::NodeHandle nh;
    ros::Subscriber sub_fan = nh.subscribe("B_infer_fan", 100, callback_fan);
    ros::Subscriber sub_track = nh.subscribe("B_infer_track", 1, callback_track);
    ros::spin();
    return 0;
}