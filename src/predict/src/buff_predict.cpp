#include <buff/buff_predict.h>

// class buff
const int max_v = 4;       //最大旋转速度(rad/s)
const double fan_length = 0.7; //大符臂长(R字中心至装甲板中心)
bool is_last_target_exists;
int last_timestamp;
Point2i last_roi_center;

CoordSolver coordsolver;
BuffPredictor predictor;
rm_msgs::B_update Omsg;
ros::Publisher pub;

// drawpred
cv::Point2f center2d_src;

BuffPredictor::BuffPredictor()
{
    is_params_confirmed = false;
    params[0] = 0;
    params[1] = 0;
    params[2] = 0;
    params[3] = 0;

    // YAML::Node config = YAML::LoadFile(pf_path);
    // pf_param_loader.initParam(config, "buff");
}
BuffPredictor::~BuffPredictor()
{
}

/**
 * @brief 预测
 * @param speed 旋转速度
 * @param dist 距离
 * @param timestamp 时间戳
 * @param result 结果输出
 * @return 是否预测成功
*/
bool BuffPredictor::predict(double speed, double dist, int timestamp, double &result)
{
    auto t1=std::chrono::steady_clock::now();
    TargetInfo target = {speed, dist, timestamp};
    if (mode != last_mode)
    {
        last_mode = mode;
        history_info.clear();
        // pf.initParam(pf_param_loader);
        is_params_confirmed = false;
    }
    //当时间跨度过长视作目标已更新，需清空历史信息队列
    if (history_info.size() == 0 || target.timestamp - history_info.front().timestamp >= max_timespan)
    {
        history_info.clear();
        history_info.push_back(target);
        params[0] = 0;
        params[1] = 0; 
        params[2] = 0; 
        params[3] = 0;
        // pf.initParam(pf_param_loader);
        last_target = target;
        is_params_confirmed = false;
        return false;
    }
    //输入数据前进行滤波
    // auto is_ready = pf.is_ready;
    Eigen::VectorXd measure(1);
    measure<<speed;
    // pf.update(measure);

    // if (is_ready)
    // {
        // auto predict = pf.predict();
        // cout<<predict<<" : "<<speed<<endl;
        // target.speed = predict[0];
    // }
    // cout<<"Mode :"<<mode<<endl;
    auto deque_len = 0;
    if (mode == 0)
    {
        deque_len = history_deque_len_uniform;
    }
    else if (mode == 1)
    {
        if (!is_params_confirmed)
            deque_len = history_deque_len_cos;
        else
            deque_len = history_deque_len_phase;
    }

    if (history_info.size() < deque_len)    
    {
        history_info.push_back(target);
        last_target = target;
        return false;
    }
    else if (history_info.size() == deque_len)
    {
        history_info.pop_front();
        history_info.push_back(target);
    }
    else if (history_info.size() > deque_len)
    {
        while(history_info.size() >= deque_len)
            history_info.pop_front();
        history_info.push_back(target);
    }


    //计算旋转方向
    double rotate_speed_sum = 0;
    int rotate_sign;
    for (auto target_info : history_info)
        rotate_speed_sum += target_info.speed;
    auto mean_velocity = rotate_speed_sum / history_info.size();
    // cout<<rotate_speed_sum<<endl;

    //TODO:小符模式不需要额外计算,也可增加判断，小符模式给定恒定转速进行击打
    if (mode == 0)
    {
        params[3] = mean_velocity;
    }
    //若为大符
    else if (mode == 1)
    {
        //拟合函数: f(x) = a * sin(ω * t + θ) + b， 其中a， ω， θ需要拟合.
        //参数未确定时拟合a， ω， θ
        if (!is_params_confirmed)
        {
            ceres::Problem problem;
            ceres::Solver::Options options;
            ceres::Solver::Summary summary;       // 优化信息
            double params_fitting[4] = {1, 1, 1, mean_velocity};
            

            //旋转方向，逆时针为正
            if (rotate_speed_sum / fabs(rotate_speed_sum) >= 0)
                rotate_sign = 1;
            else
                rotate_sign = -1;

            for (auto target_info : history_info)
            {
                problem.AddResidualBlock (     // 向问题中添加误差项
                // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                    new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4> ( 
                        new CURVE_FITTING_COST ((float)(target_info.timestamp) / 1e3,
                                                                    target_info.speed  * rotate_sign)
                    ),
                    new ceres::CauchyLoss(0.5),
                    params_fitting                 // 待估计参数
                );
            }

            //设置上下限
            //FIXME:参数需根据场上大符实际调整
            problem.SetParameterLowerBound(params_fitting,0,0.7);
            problem.SetParameterUpperBound(params_fitting,0,1.2);
            problem.SetParameterLowerBound(params_fitting,1,1.6);
            problem.SetParameterUpperBound(params_fitting,1,2.2);
            problem.SetParameterLowerBound(params_fitting,2,-CV_PI);
            problem.SetParameterUpperBound(params_fitting,2,CV_PI);
            problem.SetParameterLowerBound(params_fitting,3,0.5);
            problem.SetParameterUpperBound(params_fitting,3,2.5);

            ceres::Solve(options, &problem, &summary);
            double params_tmp[4] = {params_fitting[0] * rotate_sign, params_fitting[1], params_fitting[2], params_fitting[3] * rotate_sign};
            auto rmse = evalRMSE(params_tmp);
            if (rmse > max_rmse)
            {   
                cout<<summary.BriefReport()<<endl;
                return false;
            }
            else
            {
                params[0] = params_fitting[0] * rotate_sign;
                params[1] = params_fitting[1];
                params[2] = params_fitting[2];
                params[3] = params_fitting[3] * rotate_sign;
                is_params_confirmed = true;
            }
        }
        //参数确定时拟合θ
        else
        {            
            ceres::Problem problem;
            ceres::Solver::Options options;
            ceres::Solver::Summary summary;       // 优化信息
            double phase;

            for (auto target_info : history_info)
            {
                problem.AddResidualBlock(     // 向问题中添加误差项
                // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                    new ceres::AutoDiffCostFunction<CURVE_FITTING_COST_PHASE, 1, 1> ( 
                        new CURVE_FITTING_COST_PHASE ((float)(target_info.timestamp) / 1e3,
                                                                    (target_info.speed - params[3]) * rotate_sign, params[0], params[1], params[3])
                    ),
                    new ceres::CauchyLoss(1e1),
                    &phase                 // 待估计参数
                );
            }

            //设置上下限
            problem.SetParameterLowerBound(&phase,0,-CV_PI);
            problem.SetParameterUpperBound(&phase,0,CV_PI);

            ceres::Solve(options, &problem, &summary);
            double params_new[4] = {params[0], params[1], phase, params[3]};
            auto old_rmse = evalRMSE(params);
            auto new_rmse = evalRMSE(params_new);
            if (new_rmse < old_rmse)
            {   
                LOG(INFO)<<"[BUFF_PREDICT]Params Updated! RMSE: "<<new_rmse;
                params[2] = phase;
            }
            cout<<"RMSE:"<<new_rmse<<endl;
        }
    }

    for (auto param : params)
        cout<<param<<" ";
    cout<<"\n"<<"--------------------"<<endl;
    int delay = (mode == 1 ? delay_big : delay_small);
    float delta_time_estimate = ((double)dist / bullet_speed) * 1e3 + delay;
    // cout<<"ETA:"<<delta_time_estimate<<endl;
    float timespan = history_info.back().timestamp;
    // delta_time_estimate = 0;
    float time_estimate = delta_time_estimate + timespan;
    // cout<<delta_time_estimate<<endl;     
    result = calcAimingAngleOffset(params, timespan / 1e3, time_estimate / 1e3, mode);
    last_target = target;

    return true;
}
/**
 * @brief 计算角度提前量
 * @param params 拟合方程参数
 * @param t0 积分下限
 * @param t1 积分上限
 * @param mode 模式
 * @return 角度提前量(rad)
*/
double BuffPredictor::calcAimingAngleOffset(double params[4], double t0, double t1 , int mode)
{
    auto a = params[0];
    auto omega = params[1];
    auto theta = params[2];
    auto b = params[3]; 
    double theta1;
    double theta0;
    // cout<<"t1: "<<t1<<endl;
    // cout<<"t0: "<<t0<<endl;
    //f(x) = a * sin(ω * t + θ) + b
    //对目标函数进行积分
    if (mode == 0)//适用于小符模式
    {
        theta0 = b * t0;
        theta1 = b * t1;
    }
    else
    {
        theta0 = (b * t0 - (a / omega) * cos(omega * t0 + theta));
        theta1 = (b * t1 - (a / omega) * cos(omega * t1 + theta));
    }
    // cout<<(theta1 - theta0) * 180 / CV_PI<<endl;
    return theta1 - theta0;
}

/**
 * @brief 滑窗滤波
 * 
 * @param start_idx 开始位置 
 * @return double 滤波结果
 */
inline double BuffPredictor::shiftWindowFilter(int start_idx=0)
{
    //TODO:修改传入参数，由start_idx改为max_iter
    //计算最大迭代次数
    auto max_iter = int(history_info.size() - start_idx) - window_size + 1;
    // cout<<max_iter<<endl;

    if (max_iter <= 0 || start_idx < 0)
        return history_info.back().speed;
    // cout<<start_idx<<":"<<history_info.at(start_idx).speed<<endl;
    // cout<<start_idx + 1<<":"<<history_info.at(start_idx + 1).speed<<endl;
    // cout<<start_idx + 2<<":"<<history_info.at(start_idx + 2).speed<<endl;
    // cout<<start_idx + 3<<":"<<history_info.at(start_idx + 3).speed<<endl;
    
    double total_sum = 0;
    // cout<<max_iter<<endl;
    for (int i = 0; i < max_iter; i++)
    {
        // cout<<"i:"<<i<<endl;
        double sum = 0;
        for (int j = 0; j < window_size; j++)
            sum += history_info.at(start_idx + i + j).speed;
        total_sum += sum / window_size;
        // cout<<total_sum<<endl;
    }
    return total_sum / max_iter;
}

/**
 * @brief 设置弹速
 * 
 * @param speed 传入弹速
 * @return true 
 * @return false 
 */
bool BuffPredictor::setBulletSpeed(double speed)
{
    bullet_speed = speed;
    return true;
}

/**
 * @brief 计算RMSE指标
 * 
 * @param params 参数首地址指针
 * @return RMSE值
 */
double BuffPredictor::evalRMSE(double params[4])
{
    double rmse_sum = 0;
    double rmse = 0;
    for (auto target_info : history_info)
    {
        auto t = (float)(target_info.timestamp) / 1e3;
        auto pred = params[0] * sin (params[1] * t + params[2]) + params[3];
        auto measure = target_info.speed;
        rmse_sum+=pow((pred - measure),2);
    }
    rmse = sqrt(rmse_sum / history_info.size());
    return rmse;
}

/**
 * @brief 计算RMSE指标
 * 
 * @param params 参数首地址指针
 * @return RMSE值 
 */
double BuffPredictor::evalMAPE(double params[4])
{
    double mape_sum = 0;
    double mape = 0;
    for (auto target_info : history_info)
    {
        auto t = (float)(target_info.timestamp) / 1e3;
        auto pred = params[0] * sin (params[1] * t + params[2]) + params[3];
        auto measure = target_info.speed;

        mape_sum+=abs((measure - pred) / measure);
        // cout<<abs((measure - pred) / measure)<<endl;
    }
    // mape = mape_sum / history_info.size() * 100;
    // cout<<mape<<endl;
    return mape;
}

void callback_predict(const rm_msgs::B_track_predict::ConstPtr& Imsg)
{
    Eigen::Matrix3d rmat_imu;
    Eigen::Matrix3d target_rmat;
    Eigen::Matrix3d last_fan_rmat_transpose;
    Eigen::Vector3d target_armor3d_world;
    Eigen::Vector3d target_centerR3d_world;
    VisionData data;
    target_armor3d_world[0] = Imsg->target_armor3d_world.x;
    target_armor3d_world[1] = Imsg->target_armor3d_world.y;
    target_armor3d_world[2] = Imsg->target_armor3d_world.z;
    target_centerR3d_world[0] = Imsg->target_centerR3d_world.x;
    target_centerR3d_world[1] = Imsg->target_centerR3d_world.y;
    target_centerR3d_world[2] = Imsg->target_centerR3d_world.z;
    rmat_imu << Imsg->rmat_imu[0],Imsg->rmat_imu[1],Imsg->rmat_imu[2],
                Imsg->rmat_imu[3],Imsg->rmat_imu[4],Imsg->rmat_imu[5],
                Imsg->rmat_imu[6],Imsg->rmat_imu[7],Imsg->rmat_imu[8];
    target_rmat <<  Imsg->target_rmat[0],Imsg->target_rmat[1],Imsg->target_rmat[2],
                    Imsg->target_rmat[3],Imsg->target_rmat[4],Imsg->target_rmat[5],
                    Imsg->target_rmat[6],Imsg->target_rmat[7],Imsg->target_rmat[8];
    last_fan_rmat_transpose <<  Imsg->last_fan_rmat_transpose[0],Imsg->last_fan_rmat_transpose[1],Imsg->last_fan_rmat_transpose[2],
                                Imsg->last_fan_rmat_transpose[3],Imsg->last_fan_rmat_transpose[4],Imsg->last_fan_rmat_transpose[5],
                                Imsg->last_fan_rmat_transpose[6],Imsg->last_fan_rmat_transpose[7],Imsg->last_fan_rmat_transpose[8];
    double theta_offset = 0;
    ///------------------------进行预测----------------------------
    if (Imsg->mode == 3) // 进入小能量机关识别模式
        predictor.mode = 0;
    else if (Imsg->mode == 4) // 进入大能量机关识别模式
        predictor.mode = 1;
    // cout<<src.mode<<":"<<predictor.mode<<endl;
    // cout<<mean_rotate_speed<<endl;
    if (!predictor.predict(Imsg->mean_rotate_speed, Imsg->mean_r_center_norm, Imsg->src_timestamp, theta_offset))
    {
        data = {(float)0, (float)0, (float)0, 0, 0, 0, 1};
        return ;
    }
    ///------------------------计算击打点----------------------------
    //将角度转化至[-PI,PI范围内]
    // theta_offset = rangedAngleRad(theta_offset);
    //由offset生成欧拉角和旋转矩阵
    Eigen::Vector3d hit_point_world = {sin(theta_offset) * fan_length, (cos(theta_offset) - 1) * fan_length,0};
    Eigen::Vector3d hit_point_cam = {0,0,0};
    //Pc = R * Pw + T
    hit_point_world = (target_rmat * hit_point_world) + target_armor3d_world;
    hit_point_cam = coordsolver.worldToCam(hit_point_world, rmat_imu);
    Eigen::Vector3d r_center_cam = coordsolver.worldToCam(target_centerR3d_world, rmat_imu);
    center2d_src = coordsolver.reproject(r_center_cam);
    auto target2d = coordsolver.reproject(hit_point_cam);

    auto angle = coordsolver.getAngle(hit_point_cam, rmat_imu);

    //-----------------判断扇叶是否发生切换-------------------------
    bool is_switched = false;
    auto delta_t = Imsg->src_timestamp - last_timestamp;
    auto relative_rmat = last_fan_rmat_transpose * target_rmat;
    //TODO:使用点乘判断旋转方向
    auto angle_axisd = Eigen::AngleAxisd(relative_rmat);
    auto rotate_speed = (angle_axisd.angle()) / delta_t * 1e3; // 计算角速度(rad/s)
    if (abs(rotate_speed) > max_v)
        is_switched = true;

    // lost_cnt = 0;
    last_roi_center = center2d_src;
    last_timestamp = Imsg->src_timestamp;
    // is_last_target_exists = true;

    //若预测出错取消本次数据发送
    if (isnan(angle[0]) || isnan(angle[1]))
    {
        data = {(float)0, (float)0, (float)0, 0, 0, 0, 1};
        return ;
    }
    
    auto time_predict = std::chrono::steady_clock::now();
    // double dr_full_ms = std::chrono::duration<double,std::milli>(time_predict - time_start).count();
    // double dr_crop_ms = std::chrono::duration<double,std::milli>(time_crop - time_start).count();
    // double dr_infer_ms = std::chrono::duration<double,std::milli>(time_infer - time_crop).count();
    // double dr_predict_ms = std::chrono::duration<double,std::milli>(time_predict - time_infer).count();

    data = {(float)angle[1], (float)angle[0], (float)hit_point_cam.norm(), is_switched, 1, 1, 1};

    // pub
    Omsg.last_roi_center.x = center2d_src.x;
    Omsg.last_roi_center.y = center2d_src.y;
    Omsg.is_last_target_exists = Imsg->is_last_target_exists;
    ros::NodeHandle nh;
    pub = nh.advertise<rm_msgs::B_update>("B_update", 10);
    pub.publish(Omsg);
    return ;
}

void imageCallback(const sensor_msgs::ImageConstPtr& Imsg)
{
    cv::Mat img = cv_bridge::toCvShare(Imsg, "bgr8")->image;
    line(img, Point2f(img.size().width / 2, 0), Point2f(img.size().width / 2, img.size().height), Scalar(0,255,0), 1);
    line(img, Point2f(0, img.size().height / 2), Point2f(img.size().width, img.size().height / 2), Scalar(0,255,0), 1);
    std::cout << "center:" << center2d_src.x << " " << center2d_src.y << endl;
    circle(img, center2d_src, 5, Scalar(255, 0, 255), 2);
    // circle(img, target2d, 5, Scalar(255, 255, 255), 2);
    // putText(img, fmt::format("FPS: {}",int(1000 / dr_full_ms)), {10, 25}, FONT_HERSHEY_SIMPLEX, 1, {0,255,0});

    cv::imshow("IMG", img);
    cv::waitKey(1);
    return ;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "buff_predict"); // 初始化ROS节点
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("B_track_predict", 100, callback_predict);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_img = it.subscribe("images", 10, imageCallback);
    ros::spin();
    return 0;
}