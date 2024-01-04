#include <armor/armor_tracker.h>

std::multimap<string, ArmorTracker> trackers_map;      //预测器Map
std::map<string,int> new_armors_cnt_map;    //装甲板计数map，记录新增装甲板数
std::map<string,SpinHeading> spin_status_map;    //反小陀螺，记录该车小陀螺状态

const int max_delta_t = 50;                //使用同一预测器的最大时间间隔(ms)
const int armor_type_wh_thres = 2.8;      //大小装甲板长宽比阈值
const double armor_roi_expand_ratio_width = 1;
const double armor_roi_expand_ratio_height = 2;
const double armor_conf_high_thres = 0.82;  //置信度大于该值的装甲板直接采用
const int max_dead_buffer = 2;              //允许因击打暂时熄灭的装甲板的出现次数
const double max_delta_dist = 0.3;          //两次预测间最大速度(m/s)
const int hero_danger_zone = 99;       //英雄危险距离阈值，检测到有小于该距离的英雄直接开始攻击

int dead_buffer_cnt;
bool is_last_target_exists;
int lost_cnt;
int src_timestamp;
int prev_timestamp;

Eigen::Matrix3d rmat_imu = Eigen::Matrix3d::Identity();
Armor last_armor;
std::vector<Armor> last_armors;
std::vector<Armor> armors;
CoordSolver coordsolver;

/**
 * @brief 选择击打车辆ID
 * 
 * @param armors 
 * @return int 
 */
int chooseTargetID(vector<Armor> &armors, int timestamp)
{
    //TODO:自瞄逻辑修改
    bool is_last_id_exists = false;
    int target_id;
    //该选择逻辑主要存在两层约束:
    //英雄约束与上次目标约束
    //若检测到危险距离内的英雄直接退出循环
    //若检测到存在上次击打目标,时间较短,且该目标运动较小,则将其选为候选目标,若遍历结束未发现危险距离内的英雄则将其ID选为目标ID.
    for (auto armor : armors)
    {
        //FIXME:该处需根据兵种修改
        //若视野中存在英雄且距离小于危险距离，直接选为目标
        if (armor.id == 1 && armor.center3d_world.norm() <= hero_danger_zone)
        {
            return armor.id;
        }
        //若存在上次击打目标,时间较短,且该目标运动较小则将其选为候选目标,若遍历结束未发现危险距离内的英雄则将其ID选为目标ID.
        else if (armor.id == last_armor.id && abs(armor.area - last_armor.area) / (float)armor.area < 0.3 && abs(timestamp - prev_timestamp) < 30)
        {
            is_last_id_exists = true;
            target_id = armor.id;
        }
    }
    //若不存在则返回面积最大的装甲板序号，即队列首元素序号
    if (is_last_id_exists)
        return target_id;
    else
        return (*armors.begin()).id;
}

/**
 * @brief 构造一个ArmorTracker对象
 * 
 * @param src Armor对象
 */
ArmorTracker::ArmorTracker(Armor src, int src_timestamp)
{
    last_armor = src;
    last_timestamp = src_timestamp;
    key = src.key;
    is_initialized = false;
    hit_score = 0;
    history_info.push_back(src);
    calcTargetScore();
}
//TODO:改变目标距中心点计算方式,可以考虑通过相机内参,xyz偏移量,yaw,pitch偏移量计算出目前瞄准点
/**
 * @brief 计算该Tracker作为目标的可能性分数
 * 
 * @return true 
 * @return false 
 */
bool ArmorTracker::calcTargetScore()
{
    vector<Point2f> points;
    float rotate_angle;
    // auto horizonal_dist_to_center = abs(last_armor.center2d.x - 640);


    RotatedRect rotated_rect = last_armor.rrect;
    //调整角度至0-90度(越水平角度越小)
    if (rotated_rect.size.width > rotated_rect.size.height)
        rotate_angle = rotated_rect.angle;
    else
        rotate_angle = 90 - rotated_rect.angle;
    
    //计算分数
    //使用log函数压缩角度权值范围
    hit_score = log(0.15 * (90 - rotate_angle) + 10) * (last_armor.area);
    // cout << "hit_socre: " <<rotate_angle<<" "<<" : "<<last_armor.area<<" "<< hit_score << endl;
    return true;
}

/**
 * @brief 更新Tracker信息
 * 
 * @param new_armor 该Tracker最新装甲板位置
 * @param new_timestamp 目前时间戳 
 * @return true 
 * @return false 
 */
bool ArmorTracker::update(Armor new_armor, int new_timestamp)
{
    if (history_info.size() <= max_history_len)
    {
        history_info.push_back(new_armor);
    }
    else
    {
        history_info.pop_front();
        history_info.push_back(new_armor);
    }

    is_initialized = true;
    prev_armor = last_armor;
    prev_timestamp = last_timestamp;
    last_armor = new_armor;
    last_timestamp = new_timestamp;

    calcTargetScore();
    
    return true;
}

void callback_track(const rm_msgs::A_infer_track::ConstPtr& Imsg)
{
    src_timestamp = Imsg->src_timestamp;
    std::cout << "track subscribed! " << src_timestamp << endl;

    // ///------------------------将对象排序，保留面积较大的对象---------------------------------
    // sort(objects.begin(),objects.end(),[](ArmorObject& prev, ArmorObject& next)
    //                                 {return prev.area > next.area;});
    // //若对象较多保留前按面积排序后的前max_armors个
    // if (objects.size() > max_armors)
    //     objects.resize(max_armors);
    //若无合适装甲板
    if (armors.empty())
    {
#ifdef SHOW_AIM_CROSS
        line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), Scalar(0,255,0), 1);
        line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), Scalar(0,255,0), 1);
#endif //SHOW_AIM_CROSS
#ifdef SHOW_IMG
        namedWindow("dst",0);
        imshow("dst",src.img);
        waitKey(1);
#endif //SHOW_IMG
#ifdef USING_SPIN_DETECT
        updateSpinScore();
#endif //USING_SPIN_DETECT
        lost_cnt++;
        is_last_target_exists = false;
        // data = {(float)0, (float)0, (float)0, 0, 0, 0, 1};
        LOG(WARNING) <<"[AUTOAIM] No available armor exists!";
        return ;
    }

    ///------------------------生成/分配ArmorTracker----------------------------
    new_armors_cnt_map.clear();
    //为装甲板分配或新建最佳ArmorTracker
    //注:将不会为灰色装甲板创建预测器，只会分配给现有的预测器
    for (auto armor = armors.begin(); armor != armors.end(); ++armor)
    {
        //当装甲板颜色为灰色且当前dead_buffer小于max_dead_buffer
        string tracker_key;
        if ((*armor).color == 2)
        {
            if (dead_buffer_cnt >= max_dead_buffer)
                continue;
            
            // if (Imsg->color == 0)
            //     tracker_key = "B" + to_string((*armor).id);
            // if (Imsg->color == 1)
            //     tracker_key = "R" + to_string((*armor).id);
        }
        else
        {
            tracker_key = (*armor).key;
        }

        auto predictors_with_same_key = trackers_map.count(tracker_key);
        //当不存在该类型装甲板ArmorTracker且该装甲板Tracker类型不为灰色装甲板
        if (predictors_with_same_key == 0 && (*armor).color != 2)
        {
            ArmorTracker tracker((*armor), src_timestamp);
            auto target_predictor = trackers_map.insert(make_pair((*armor).key, tracker));
            new_armors_cnt_map[(*armor).key]++;
        }
        //当存在一个该类型ArmorTracker
        else if (predictors_with_same_key == 1)
        {
            auto candidate = trackers_map.find(tracker_key);
            auto delta_t = src_timestamp - (*candidate).second.last_timestamp;
            auto delta_dist = ((*armor).center3d_world - (*candidate).second.last_armor.center3d_world).norm();
            // auto iou = (*candidate).second.last_armor.roi & (*armor)
            // auto velocity = (delta_dist / delta_t) * 1e3;
            //若匹配则使用此ArmorTracker
            if (delta_dist <= max_delta_dist && delta_t > 0 && (*candidate).second.last_armor.roi.contains((*armor).center2d))
            {
                (*candidate).second.update((*armor), src_timestamp);
            }
            //若不匹配则创建新ArmorTracker
            else if ((*armor).color != 2)
            {
                ArmorTracker tracker((*armor), src_timestamp);
                trackers_map.insert(make_pair((*armor).key, tracker));
                new_armors_cnt_map[(*armor).key]++;
            }
        }
        //当存在多个该类型装甲板ArmorTracker
        else
        {
            //1e9无实际意义，仅用于以非零初始化
            double min_delta_dist = 1e9;
            int min_delta_t = 1e9;
            bool is_best_candidate_exist = false;
            std::multimap<string, ArmorTracker>::iterator best_candidate;
            auto candiadates = trackers_map.equal_range(tracker_key);
            //遍历所有同Key预测器，匹配速度最小且更新时间最近的ArmorTracker
            for (auto iter = candiadates.first; iter != candiadates.second; ++iter)
            {
                auto delta_t = src_timestamp - (*iter).second.last_timestamp;
                auto delta_dist = ((*armor).center3d_world - (*iter).second.last_armor.center3d_world).norm();
                auto velocity = (delta_dist / delta_t) * 1e3;
                
                if ((*iter).second.last_armor.roi.contains((*armor).center2d) && delta_t > 0)
                {
                    if (delta_dist <= max_delta_dist && delta_dist <= min_delta_dist &&
                     delta_t <= min_delta_t)
                    {
                        min_delta_t = delta_t;
                        min_delta_dist = delta_dist;
                        best_candidate = iter;
                        is_best_candidate_exist = true;
                    }
                }
            }
            if (is_best_candidate_exist)
            {
                auto velocity = min_delta_dist;
                auto delta_t = min_delta_t;
                (*best_candidate).second.update((*armor), src_timestamp);
            }
            else if ((*armor).color != 2)
            {
                ArmorTracker tracker((*armor), src_timestamp);
                trackers_map.insert(make_pair((*armor).key, tracker));
                new_armors_cnt_map[(*armor).key]++;
            }

        }
    }
    if (trackers_map.size() != 0)
    {
        //维护预测器Map，删除过久之前的装甲板
        for (auto iter = trackers_map.begin(); iter != trackers_map.end();)
        {
            //删除元素后迭代器会失效，需先行获取下一元素
            auto next = iter;
            // cout<<(*iter).second.last_timestamp<<"  "<<src_timestamp<<endl;
            if ((src_timestamp - (*iter).second.last_timestamp) > max_delta_t)
                next = trackers_map.erase(iter);
            else
                ++next;
            iter = next;
        }
    }
    // cout<<"::"<<predictors_map.size()<<endl;
    // for (auto member : new_armors_cnt_map)
    //     cout<<member.first<<" : "<<member.second<<endl;
#ifdef USING_SPIN_DETECT
    ///------------------------检测装甲板变化情况,计算各车陀螺分数----------------------------
    for (auto cnt : new_armors_cnt_map)
    {
        //只在该类别新增装甲板时数量为1时计算陀螺分数
        if (cnt.second == 1)
        {
            auto same_armors_cnt = trackers_map.count(cnt.first);
            if (same_armors_cnt == 2)
            {
                // cout<<"1"<<endl;
                //遍历所有同Key预测器，确定左右侧的Tracker
                ArmorTracker *new_tracker = nullptr;
                ArmorTracker *last_tracker = nullptr;
                double last_armor_center;
                double last_armor_timestamp;
                double new_armor_center;
                double new_armor_timestamp;
                int best_prev_timestamp = 0;    //候选ArmorTracker的最近时间戳
                auto candiadates = trackers_map.equal_range(cnt.first);
                for (auto iter = candiadates.first; iter != candiadates.second; ++iter)
                {
                    //若未完成初始化则视为新增tracker
                    if (!(*iter).second.is_initialized && (*iter).second.last_timestamp == src_timestamp)
                    {
                        new_tracker = &(*iter).second;
                    }
                    else if ((*iter).second.last_timestamp > best_prev_timestamp && (*iter).second.is_initialized)
                    {
                        best_prev_timestamp = (*iter).second.last_timestamp;
                        last_tracker = &(*iter).second;
                    }
                    
                }
                if (new_tracker != nullptr && last_tracker != nullptr)
                {
                    new_armor_center = new_tracker->last_armor.center2d.x;
                    new_armor_timestamp = new_tracker->last_timestamp;
                    last_armor_center = last_tracker->last_armor.center2d.x;
                    last_armor_timestamp = last_tracker->last_timestamp;
                    auto spin_movement = new_armor_center - last_armor_center;
                    // auto delta_t = 
                    LOG(INFO)<<"[SpinDetection] Candidate Spin Movement Detected : "<<cnt.first<<" : "<<spin_movement;
                    if (abs(spin_movement) > 10 && new_armor_timestamp == src_timestamp && last_armor_timestamp == src_timestamp)
                    {

                        //若无该元素则插入新元素
                        if (spin_score_map.count(cnt.first) == 0)
                        {
                            spin_score_map[cnt.first] = 1000 * spin_movement / abs(spin_movement);
                        }
                        //若已有该元素且目前旋转方向与记录不同,则对目前分数进行减半惩罚
                        else if (spin_movement * spin_score_map[cnt.first] < 0)
                        {
                            spin_score_map[cnt.first] *= 0.5;
                        }
                        //若已有该元素则更新元素
                        else
                        {
                            spin_score_map[cnt.first] = anti_spin_max_r_multiple * spin_score_map[cnt.first];
                        }
                    }
                }
            }
        }
    }
    ///------------------更新反陀螺socre_map，更新各车辆陀螺状态-----------------------------
    updateSpinScore();
    // cout<<"-----------------------"<<endl;
    // for (auto status : spin_status_map)
    // {
    //     cout<<status.first<<" : "<<status.second<<endl;
    // }
#endif //USING_SPIN_DETECT
    ///-----------------------------判断击打车辆------------------------------------------
    auto target_id = chooseTargetID(armors, src_timestamp);
    string target_key;
    // if (detect_color == BLUE)
    //     target_key = "B" + to_string(target_id);
    // else if (detect_color == RED)
    //     target_key = "R" + to_string(target_id);
    // cout<<target_key<<endl;
    ///-----------------------------判断该装甲板是否有可用Tracker------------------------------------------
    if (trackers_map.count(target_key) == 0)
    {
#ifdef SHOW_AIM_CROSS
        line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), Scalar(0,255,0), 1);
        line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), Scalar(0,255,0), 1);
#endif //SHOW_AIM_CROSS
#ifdef SHOW_IMG
        namedWindow("dst",0);
        imshow("dst",src.img);
        waitKey(1);
#endif //SHOW_IMG
        lost_cnt++;
        is_last_target_exists = false;
        // data = {(float)0, (float)0, (float)0, 0, 0, 0, 1};
        LOG(WARNING) <<"[AUTOAIM] No available tracker exists!";
        return ;
    }
    auto ID_candiadates = trackers_map.equal_range(target_key);
    ///---------------------------获取最终装甲板序列---------------------------------------
    bool is_target_spinning;
    Armor target;
    Eigen::Vector3d aiming_point;
    std::vector<ArmorTracker*> final_trackers;
    std::vector<Armor> final_armors;
    //TODO:反陀螺防抖(增加陀螺模式与常规模式)
    //若目标处于陀螺状态，预先瞄准目标中心，待预测值与该点距离较近时开始击打
    SpinHeading spin_status;
    if (spin_status_map.count(target_key) == 0)
    {
        spin_status = UNKNOWN;
        is_target_spinning = false;
    }
    else
    {
        spin_status = spin_status_map[target_key];
        if (spin_status != UNKNOWN)
            is_target_spinning = true;
        else
            is_target_spinning = false;
    }
    //pub
    // rm_msgs::A_track_predict Omsg;
    // Omsg.mode = mode;
    // Omsg.mean_rotate_speed = mean_rotate_speed;
    // Omsg.mean_r_center_norm = mean_r_center.norm();
    // Omsg.src_timestamp = Imsg->src_timestamp;
    // Omsg.target_armor3d_world.x = target.armor3d_world[0];
    // Omsg.target_armor3d_world.y = target.armor3d_world[1];
    // Omsg.target_armor3d_world.z = target.armor3d_world[2];
    // Omsg.target_centerR3d_world.x = target.centerR3d_world[0];
    // Omsg.target_centerR3d_world.y = target.centerR3d_world[1];
    // Omsg.target_centerR3d_world.z = target.centerR3d_world[2];
    // for(int i = 0; i < 9; i++){
    //     Omsg.rmat_imu[i] = rmat_imu(i);
    //     Omsg.target_rmat[i] = target.rmat(i);
    //     Omsg.last_fan_rmat_transpose[i] = last_fan_rmat_transpose(i);
    // }
    // Omsg.is_last_target_exists = true;
    // ros::NodeHandle nh;
    // std::cout<<"The target is going to be published !"<<endl;
    // pub = nh.advertise<rm_msgs::B_track_predict>("B_track_predict", 10);
    // pub.publish(Omsg);
    //update
    // is_last_target_exists = true;
    // last_fan = target;
    // fans.clear();
    return ;
}

void callback_armor(const rm_msgs::A_infer_armor::ConstPtr& Imsg)
{
    Armor armor;
    armor.id = Imsg->cls;
    armor.color = Imsg->color;
    armor.conf = Imsg->prob;
    if (Imsg->color == 0)
        armor.key = "B" + string(Imsg->cls == 0 ? "Activated" : "Target");
    if (Imsg->color == 1)
        armor.key = "R" + string(Imsg->cls == 0 ? "Activated" : "Target");
    //生成顶点与装甲板二维中心点
    armor.apex2d[0].x = Imsg->apex_0.x;
    armor.apex2d[0].y = Imsg->apex_0.y;
    armor.apex2d[1].x = Imsg->apex_1.x;
    armor.apex2d[1].y = Imsg->apex_1.y;
    armor.apex2d[2].x = Imsg->apex_2.x;
    armor.apex2d[2].y = Imsg->apex_2.y;
    armor.apex2d[3].x = Imsg->apex_3.x;
    armor.apex2d[3].y = Imsg->apex_3.y;
    for(int i = 0; i < 4; i++)
        armor.apex2d[i] += Point2f((float)Imsg->roi_offset.x,(float)Imsg->roi_offset.y);
    // std::cout<<"armor subscribed! "<<src_timestamp<<" "<<armor.id<<endl;
    Point2f apex_sum;
    for(auto apex : armor.apex2d)
        apex_sum +=apex;
    armor.center2d = apex_sum / 4.f;
    // 生成装甲板旋转矩形和ROI
    std::vector<Point2f> points_pic(armor.apex2d, armor.apex2d + 4);
    RotatedRect points_pic_rrect = minAreaRect(points_pic);        
    armor.rrect = points_pic_rrect;
    auto bbox = points_pic_rrect.boundingRect();
    auto x = bbox.x - 0.5 * bbox.width * (armor_roi_expand_ratio_width - 1);
    auto y = bbox.y - 0.5 * bbox.height * (armor_roi_expand_ratio_height - 1);
    armor.roi = Rect(x,
                    y,
                    bbox.width * armor_roi_expand_ratio_width,
                    bbox.height * armor_roi_expand_ratio_height
                    );
    // 若装甲板置信度小于高阈值，需要相同位置存在过装甲板才放行
    if (armor.conf < armor_conf_high_thres){
        if (last_armors.empty()){
            return ;
        }else{
            bool is_this_armor_available = false;
            for (auto last_armor : last_armors){
                if (last_armor.roi.contains(armor.center2d)){
                    is_this_armor_available = true;
                    break;
                }
            }
            if (!is_this_armor_available){
                // cout<<"IGN"<<endl;
                return;
            }
        }
    }
    // cout<<"..."<<endl;
    //进行PnP，目标较少时采取迭代法，较多时采用IPPE
    int pnp_method;
    // if (objects.size() <= 2)
    //     pnp_method = SOLVEPNP_ITERATIVE;
    // else
        pnp_method = SOLVEPNP_IPPE;
    TargetType target_type = SMALL;
    //计算长宽比,确定装甲板类型
    auto apex_wh_ratio = max(points_pic_rrect.size.height, points_pic_rrect.size.width) /
                            min(points_pic_rrect.size.height, points_pic_rrect.size.width);
    //若大于长宽阈值或为哨兵、英雄装甲板
    //FIXME:若存在平衡步兵需要对此处步兵装甲板类型进行修改
    if (armor.id == 0 || armor.id == 1)
        target_type = BIG;
    else if (armor.id == 2 || armor.id == 3 || armor.id == 4 || armor.id == 5 || armor.id == 6)
        target_type = SMALL;
    else if (apex_wh_ratio > armor_type_wh_thres)
        target_type = BIG;
    // for (auto pic : points_pic)
    //     cout<<pic<<endl;
    auto pnp_result = coordsolver.pnp(points_pic, rmat_imu, target_type, pnp_method);
    //防止装甲板类型出错导致解算问题，距离过大或出现NAN直接跳过该装甲板
    if (pnp_result.armor_cam.norm() > 13 ||
        isnan(pnp_result.armor_cam[0]) ||
        isnan(pnp_result.armor_cam[1]) ||
        isnan(pnp_result.armor_cam[2]))
            return ;
    
    armor.type = target_type;
    armor.center3d_world = pnp_result.armor_world;
    armor.center3d_cam = pnp_result.armor_cam;
    armor.euler = pnp_result.euler;
    armor.area = Imsg->area;
    armors.push_back(armor);
    return ;
}

int main(int argc,char** argv)
{
    coordsolver.loadParam(camera_param_path,camera_name);
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "armor_tracker"); // 初始化ROS节点
    ros::NodeHandle nh;
    ros::Subscriber sub_armor = nh.subscribe("A_infer_armor", 100, callback_armor);
    ros::Subscriber sub_track = nh.subscribe("A_infer_track", 1, callback_track);
    ros::spin();
    return 0;
}