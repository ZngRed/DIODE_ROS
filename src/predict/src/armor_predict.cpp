#include <armor/armor_predict.h>

int last_timestamp;
int src_timestamp;

CoordSolver coordsolver;

ArmorPredictor::ArmorPredictor()
{
}
ArmorPredictor::~ArmorPredictor()
{
}

ArmorPredictor ArmorPredictor::generate()
{
    ArmorPredictor new_predictor;
    // new_predictor.pf_pos.initParam(pf_pos);
    // new_predictor.pf_v.initParam(pf_v);
    new_predictor.fitting_disabled = false;

    return new_predictor;
}

bool ArmorPredictor::initParam(ArmorPredictor &predictor_loader)
{
    history_info.clear();
    // pf_pos.initParam(predictor_loader.pf_pos);
    // pf_v.initParam(predictor_loader.pf_v);
    fitting_disabled = false;
    
    return true;
}

bool ArmorPredictor::initParam(string coord_path)
{
    YAML::Node config = YAML::LoadFile(coord_path);
    // pf_pos.initParam(config,"pos");
    // pf_v.initParam(config,"v");
    fitting_disabled = false;
    
    return true;
}

Eigen::Vector3d ArmorPredictor::predict(Eigen::Vector3d xyz, int timestamp)
{
    auto t1=std::chrono::steady_clock::now();
    TargetInfo target = {xyz, (int)xyz.norm(), timestamp};
    //-----------------对位置进行粒子滤波,以降低测距噪声影响-------------------------------------
    // Eigen::VectorXd measure (2);
    // measure << xyz[0], xyz[1];
    // bool is_pos_filter_ready = pf_pos.update(measure);
    // Eigen::VectorXd predict_pos_xy = pf_pos.predict();
    // Eigen::Vector3d predict_pos = {predict_pos_xy[0], predict_pos_xy[1], xyz[2]};
    //若位置粒子滤波器未完成初始化或滤波结果与目前位置相距过远,则本次不对目标位置做滤波,直接向队列压入原值
    // if (!is_pos_filter_ready || (predict_pos - xyz).norm() > 0.1)
    // {
    //     history_info.push_back(target);
    // }
    //若位置粒子滤波器已完成初始化且预测值大小恰当,则对目标位置做滤波
    // else
    // {
    //     // cout<<"FIL:"<<predict_pos[0] - xyz[0]<<endl;
    //     target.xyz[0] = predict_pos[0];
    //     target.xyz[1] = predict_pos[1];
    //     history_info.push_back(target);
    // }
    //-----------------进行滑窗滤波,备选方案,暂未使用-------------------------------------
    auto d_xyz = target.xyz - last_target.xyz;
    auto delta_t = timestamp - last_target.timestamp;
    auto last_dist = history_info.back().dist;
    auto delta_time_estimate = (last_dist / bullet_speed) * 1e3 + delay;
    auto time_estimate = delta_time_estimate + history_info.back().timestamp;
    //如速度过大,可认为为噪声干扰,进行滑窗滤波滤除
    // if (((d_xyz.norm() / delta_t) * 1e3) >= max_v)
    // {
    //     history_info.push_back(target);
    //     auto filtered_xyz = shiftWindowFilter(history_info.size() - window_size - 1);
    //     target = {filtered_xyz, (int)filtered_xyz.norm(), timestamp};
    //     history_info.pop_back();
    // }
    // auto filtered_xyz = shiftWindowFilter(history_info.size() - window_size - 1);
    // filtered_xyz << xyz[0], xyz[1], filtered_xyz[2];
    // target = {filtered_xyz, (int)filtered_xyz.norm(), timestamp};
    // history_info.pop_back();

    //---------------根据目前队列长度选用合适的滤波器------------------------------------
    //当队列长度小于3，仅更新队列
    if (history_info.size() < 4)
    {
        last_target = target;
        return xyz;
    }
    //当队列长度不足时不使用拟合
    else if (history_info.size() < min_fitting_len)
    {
        fitting_disabled = true;
    }
    //当队列时间跨度过长时不使用拟合
    else if (target.timestamp - history_info.front().timestamp >= max_timespan)
    {
        history_info.pop_front();
        fitting_disabled = true;
    }
    //其余状况下皆可以进行拟合
    else
    {
        fitting_disabled = false;
        //若队列过长，移除首元素
        if (history_info.size() > history_deque_len)
            history_info.pop_front();
    }

    
#ifdef DISABLE_FITTING
    fitting_disabled = true;
#endif //DISABLE_FITTING

    Eigen::Vector3d result = {0, 0, 0};
    // Eigen::Vector3d result_pf = {0, 0, 0};
    Eigen::Vector3d result_fitting = {0, 0, 0};
    // PredictStatus is_pf_available;
    PredictStatus is_fitting_available;
    //需注意粒子滤波使用相对时间（自上一次检测时所经过ms数），拟合使用自首帧所经过时间
    if (fitting_disabled)
    {
        // auto is_pf_available = predict_pf_run(target, result_pf, delta_time_estimate);
    }
    else
    {
        // auto get_pf_available = std::async(std::launch::async, [=, &result_pf](){return predict_pf_run(target, result_pf, delta_time_estimate);});
        auto get_fitting_available = std::async(std::launch::async, [=, &result_fitting](){return predict_fitting_run(result_fitting, time_estimate);});

        // is_pf_available = get_pf_available.get();
        is_fitting_available = get_fitting_available.get();
    }
    // if (fitting_disabled)
    // {
    //     return xyz;
    //     // auto is_pf_available = predict_pf_run(target, result_pf, delta_time_estimate);
    // }
    // else
    // {
    //     auto get_fitting_available = std::async(std::launch::async, [=, &result_fitting](){return predict_fitting_run(result_fitting, time_estimate);});

    //     is_fitting_available = get_fitting_available.get();
    // }
    // 进行融合
    if (is_fitting_available.xyz_status[0] && !fitting_disabled)
        result[0] = result_fitting[0];
    // else if (is_pf_available.xyz_status[0])
        // result[0] = result_pf[0];
    else
        result[0] = xyz[0];

    if (is_fitting_available.xyz_status[1] && !fitting_disabled)
        result[1] = result_fitting[1];
    // else if (is_pf_available.xyz_status[1])
        // result[1] = result_pf[1];
    else
        result[1] = xyz[1];

    if (is_fitting_available.xyz_status[2] && !fitting_disabled)
        result[2] = result_fitting[2];
    else
        result[2] = xyz[2];
    // result = result_pf;
    auto t2=std::chrono::steady_clock::now();
    double dr_ms=std::chrono::duration<double,std::milli>(t2-t1).count();
    // if(timestamp % 10 == 0)
    delta_time_estimate = 0;
    // cout<<dr_ms<<endl;
    // cout<<xyz<<endl;
    // result_pf = target.xyz;
#ifdef DRAW_PREDICT
    double x_offset = 400;
    double y_offset = 400;
    double z_offset = 200;
    if (cnt < 2000)
    {
        auto x = cnt * 5;
        cv::circle(pic_x,cv::Point2f((timestamp) / 10,xyz[0] * 100 + x_offset),1,cv::Scalar(0,0,255),1);
        cv::circle(pic_x,cv::Point2f((timestamp + delta_time_estimate) / 10,result_pf[0] * 100 + x_offset),1,cv::Scalar(0,255,0),1);
        cv::circle(pic_x,cv::Point2f((timestamp + delta_time_estimate) / 10,result_fitting[0] * 100 + x_offset),1,cv::Scalar(255,255,0),1);
        // cv::circle(pic_x,cv::Point2f((timestamp + delta_time_estimate) / 10,result[0]+ 200),1,cv::Scalar(255,255,255),1);


        cv::circle(pic_y,cv::Point2f((timestamp) / 10,xyz[1] * 100 + y_offset),1,cv::Scalar(0,0,255),1);
        cv::circle(pic_y,cv::Point2f((timestamp + delta_time_estimate) / 10,result_pf[1] * 100 + y_offset),1,cv::Scalar(0,255,0),1);
        cv::circle(pic_y,cv::Point2f((timestamp + delta_time_estimate) / 10,result_fitting[1] * 100 + y_offset),1,cv::Scalar(255,255,0),1);
        // cv::circle(pic_y,cv::Point2f((timestamp + delta_time_estimate) / 10,result[1]+ 200),1,cv::Scalar(255,255,255),1);

        cv::circle(pic_z,cv::Point2f((timestamp) / 10,xyz[2] * 100 + z_offset),1,cv::Scalar(0,0,255),1);
        cv::circle(pic_z,cv::Point2f((timestamp + delta_time_estimate) / 10,result_pf[2] * 100  + z_offset),1,cv::Scalar(0,255,0),1);
        cv::circle(pic_z,cv::Point2f((timestamp + delta_time_estimate) / 10,result_fitting[2] * 100 + z_offset),1,cv::Scalar(255,255,0),1);
        // cv::circle(pic_z,cv::Point2f((timestamp + delta_time_estimate) / 10,result[2]),1,cv::Scalar(255,255,255),1);
        cnt++;
    }
    cv::imshow("result_x",pic_x);
    cv::imshow("result_y",pic_y);
    cv::imshow("result_z",pic_z);
    cv::waitKey(1);
#endif //DRAW_PREDICT
    last_target = target;
    // return target.xyz;
    return result;
}

inline Eigen::Vector3d ArmorPredictor::shiftWindowFilter(int start_idx=0)
{
    //计算最大迭代次数
    auto max_iter = int(history_info.size() - start_idx) - window_size + 1;
    Eigen::Vector3d total_sum = {0, 0, 0};
    // cout<<history_info.size()<<endl;
    // cout<<max_iter<<endl;
    // cout<<start_idx<<endl;
    if (max_iter == 0 || start_idx < 0)
        return history_info.back().xyz;
    
    for (int i = 0; i < max_iter; i++)
    {
        Eigen::Vector3d sum = {0,0,0};
        for (int j = 0; j < window_size; j++)
            sum += history_info.at(start_idx + i + j).xyz;
        total_sum += sum / window_size;
    }
    // cout<<history_info.back().xyz<<endl;
    // cout<<total_sum / max_iter<<endl;
    // cout<<endl;
    return total_sum / max_iter;
}

/**
 * @brief 进行一次粒子滤波预测
 * @param target 本次预测目标信息
 * @param result 输出结果引用
 * @param time_estimated 本次预测所需的时间提前量
 * @return 预测结果状态
 * **/
// ArmorPredictor::PredictStatus ArmorPredictor::predict_pf_run(TargetInfo target, Vector3d &result, int time_estimated)
// {
//     PredictStatus is_available;
//     //采取中心差分法,使用 t, t-1, t-2时刻速度,计算t-1时刻的速度
//     auto target_prev = history_info.at(history_info.size() - 3);
//     auto target_next = target;
//     auto v_xyz = (target_next.xyz - target_prev.xyz) / (target_next.timestamp - target_prev.timestamp) * 1e3;
//     auto t = target_next.timestamp - history_info.at(history_info.size() - 2).timestamp;
// 
//     is_available.xyz_status[0] = pf_v.is_ready;
//     is_available.xyz_status[1] = pf_v.is_ready;
//     // cout<<v_xyz<<endl;
// 
//     //Update
//     Eigen::VectorXd measure (2);
//     measure << v_xyz[0], v_xyz[1];
//     pf_v.update(measure);
// 
//     //Predict
//     auto result_v = pf_v.predict();
//     cout<<measure<<endl;
//     // cout<<result_v<<endl;
//     //TODO:恢复速度预测
//     // auto predict_x = target.xyz[0];
//     // auto predict_y = target.xyz[1];
//     double predict_x;
//     double predict_y;
//     if (history_info.size() > 6)
//     {
//         predict_x = target.xyz[0] + result_v[0] * (time_estimated + t) / 1e3;
//         predict_y = target.xyz[1] + result_v[1] * (time_estimated + t) / 1e3;
//     }
//     else
//     {
//         predict_x = target.xyz[0];
//         predict_y = target.xyz[1];       
//     }
// 
//     result << predict_x, predict_y, target.xyz[2];
//     // cout<<result<<endl;
// 
//     // cout<<result<<endl;
// 
//     return is_available;
// }

ArmorPredictor::PredictStatus ArmorPredictor::predict_fitting_run(Eigen::Vector3d &result, int time_estimated)
{
    //0.1的位置使用0初始化会导致拟合结果出错
    double params_x[4] = {0,0,0,0};            // 参数的估计值
    double params_y[4] = {0,0,0,0};            // 参数的估计值

    ceres::Problem problem_x;
    ceres::Problem problem_y;

    ceres::Solver::Options options_x;
    ceres::Solver::Options options_y;

    ceres::Solver::Summary summary_x;                // 优化信息
    ceres::Solver::Summary summary_y;                // 优化信息

    options_x.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options_y.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解

    //求直流分量
    Eigen::Vector3d sum = {0,0,0};
    for (auto target_info : history_info)
    {
        sum += target_info.xyz;
    }
    auto dc = sum / history_info.size();
    // auto dc = history_info.at(history_info.size() - 1).xyz;
    params_x[0] = dc[0];
    params_y[0] = dc[1];
    
    for (auto target_info : history_info)
    {
        cout<<"T : "<<target_info.timestamp / 1e3<<" X:"<<target_info.xyz[0]<<" Y:"<<target_info.xyz[1]<<endl;
        problem_x.AddResidualBlock (     // 向问题中添加误差项
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 2> ( 
                new CURVE_FITTING_COST (target_info.timestamp / 1e3, (target_info.xyz[0] - params_x[0]))
            ),
            new ceres::CauchyLoss(0.5),            // 核函数，这里不使用，为空
            &params_x[1]                 // 待估计参数
        );
        problem_y.AddResidualBlock(     // 向问题中添加误差项 
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 2> ( 
                new CURVE_FITTING_COST (target_info.timestamp / 1e3, (target_info.xyz[1] - params_y[0]))
            ),
           new ceres::CauchyLoss(0.5),            // 核函数，这里不使用，为空
            &params_y[1]                 // 待估计参数
        );
    }
    // cout<<endl;
    // problem_x.SetParameterUpperBound(&params_x[1],0,20);
    // problem_x.SetParameterLowerBound(&params_x[1],0,-20);
    // problem_x.SetParameterUpperBound(&params_x[1],1,0.5);
    // problem_x.SetParameterLowerBound(&params_x[1],1,-0.5);
    // problem_x.SetParameterUpperBound(&params_x[1],2,CV_PI);
    // problem_x.SetParameterLowerBound(&params_x[1],2,-CV_PI);

    // problem_y.SetParameterUpperBound(&params_y[1],0,20);
    // problem_y.SetParameterLowerBound(&params_y[1],0,-20);
    // problem_y.SetParameterUpperBound(&params_y[1],1,0.5);
    // problem_y.SetParameterLowerBound(&params_y[1],1,-0.5);
    // problem_y.SetParameterUpperBound(&params_y[1],2,CV_PI);
    // problem_y.SetParameterLowerBound(&params_y[1],2,-CV_PI);

    // problem_x.SetParameterUpperBound(&params_x[1],0,5);
    // problem_x.SetParameterLowerBound(&params_x[1],0,-5);
    // problem_x.SetParameterUpperBound(&params_x[1],1,5);
    // problem_x.SetParameterLowerBound(&params_x[1],1,-5);

    // problem_y.SetParameterUpperBound(&params_y[1],0,5);
    // problem_y.SetParameterLowerBound(&params_y[1],0,-5);
    // problem_y.SetParameterUpperBound(&params_y[1],1,5);
    // problem_y.SetParameterLowerBound(&params_y[1],1,-5);
    //异步计算

    auto status_solve_x = std::async(std::launch::deferred, [&](){ceres::Solve(options_x, &problem_x, &summary_x);});
    auto status_solve_y = std::async(std::launch::deferred, [&](){ceres::Solve(options_y, &problem_y, &summary_y);});

    status_solve_x.wait();
    status_solve_y.wait();

    auto x_cost = summary_x.final_cost;
    auto y_cost = summary_y.final_cost;
    // cout<<x_cost<<endl;

    PredictStatus is_available;

    is_available.xyz_status[0] = (x_cost <= max_cost);
    is_available.xyz_status[1] = (y_cost <= max_cost);
    // cout<<z_cost<<endl;
    
    cout<<"X:"<<params_x[0]<<" "<<params_x[1]<<" "<<params_x[2]<<" "<<params_x[3]<<endl; 
    cout<<"Y:"<<params_y[0]<<" "<<params_y[1]<<" "<<params_y[2]<<" "<<params_y[3]<<endl;
    // cout<<summary_y.BriefReport()<<endl;
    // cout<<time_estimated<<endl;
    // cout<<bullet_speed<<endl;
    auto x_pred = params_x[0] + params_x[1] * (time_estimated / 1e3) + params_x[2] * pow((time_estimated / 1e3), 2);
    auto y_pred = params_y[0] + params_y[1] * (time_estimated / 1e3) + params_y[2] * pow((time_estimated / 1e3), 2);  
    // auto x_pred = params_x[0] + params_x[1] * cos(params_x[2] * (time_estimated / 1e3) + params_x[3]);
    // auto y_pred = params_y[0] + params_y[1] * cos(params_y[2] * (time_estimated / 1e3) + params_y[3]);
    // auto x_pred = params_x[0] + params_x[1] * cos(params_x[3] * (time_estimated / 1e3)) + params_x[2] * sin(params_x[3] * (time_estimated / 1e3));
    // auto y_pred = params_y[0] + params_y[1] * cos(params_y[3] * (time_estimated / 1e3)) + params_y[2] * sin(params_y[3] * (time_estimated / 1e3));

    cout<<x_pred<<" : "<<y_pred<<endl;
    cout<<"..........."<<endl;
    result = {x_pred, y_pred, dc[2]};
    return is_available;
}

bool ArmorPredictor::setBulletSpeed(double speed)
{
    bullet_speed = speed;
    return true;
}

void callback_predict(const rm_msgs::A_track_predict::ConstPtr& Imsg)
{
    // spin_status ID_candiadates final_armors final_trackers target Armor is_last_target_exists is_target_switched
    ///----------------------------------反陀螺击打---------------------------------------
    if (spin_status != UNKNOWN)
    {
        //------------------------------尝试确定旋转中心-----------------------------------
        auto available_candidates_cnt = 0;
        for (auto iter = ID_candiadates.first; iter != ID_candiadates.second; ++iter)
        {
            if ((*iter).second.last_timestamp == src_timestamp)
            {
                final_armors.push_back((*iter).second.last_armor);
                final_trackers.push_back(&(*iter).second);
            }
            else
            {
                continue;
            }
            // // 若Tracker未完成初始化，不考虑使用
            // if (!(*iter).second.is_initialized || (*iter).second.history_info.size() < 3)
            // {
            //     continue;
            // }
            // else
            // {
            //     final_trackers.push_back(&(*iter).second);
            //     available_candidates_cnt++;
            // }
        }
        // if (available_candidates_cnt == 0)
        // {
        //     cout<<"Invalid"<<endl;
        // }
        // else
        // {   //TODO:改进旋转中心识别方法
        //     //FIXME:目前在目标小陀螺时并移动时，旋转中心的确定可能存在问题，故该语句块中的全部计算结果均暂未使用
        //     //-----------------------------计算陀螺旋转半径--------------------------------------
        //     Eigen::Vector3d rotate_center_cam = {0,0,0};
        //     Eigen::Vector3d rotate_center_car = {0,0,0};
        //     for(auto tracker : final_trackers)
        //     {
        //         std::vector<Eigen::Vector3d> pts;
        //         for (auto pt : tracker->history_info)
        //         {
        //             pts.push_back(pt.center3d_world);
        //         }
        //         auto sphere = FitSpaceCircle(pts);
        //         auto radius = sphere[3];
        //         if (tracker->radius == 0)
        //             tracker->radius = radius;
        //         else//若不为初值，尝试进行半径平均以尽量误差
        //             tracker->radius = (tracker->radius + radius) / 2;
        //         //-----------------------------计算陀螺中心与预瞄点-----------------------------------
        //         //此处世界坐标系指装甲板世界坐标系，而非车辆世界坐标系
        //         Eigen::Vector3d rotate_center_world = {0,
        //                             sin(25 * 180 / CV_PI) * tracker->radius,
        //                             - cos(25 * 180 / CV_PI) * tracker->radius};
        //         auto rotMat = eulerToRotationMatrix(tracker->prev_armor.euler);
        //         //Pc = R * Pw + T
        //         rotate_center_cam = (rotMat * rotate_center_world) + tracker->prev_armor.center3d_cam;
        //         rotate_center_car += coordsolver.worldToCam(rotate_center_cam, rmat_imu);
        //     }
        //     //求解旋转中心
        //     rotate_center_car /= final_trackers.size();
        // }
        //若存在一块装甲板
        if (final_armors.size() == 1)
        {
            target = final_armors.at(0);
        }
        //若存在两块装甲板
        else if (final_armors.size() == 2)
        {
            //对最终装甲板进行排序，选取与旋转方向相同的装甲板进行更新
            sort(final_armors.begin(),final_armors.end(),[](Armor& prev, Armor& next)
                                {return prev.center3d_cam[0] < next.center3d_cam[0];});
            //若顺时针旋转选取右侧装甲板更新
            if (spin_status == CLOCKWISE)
                target = final_armors.at(1);
            //若逆时针旋转选取左侧装甲板更新
            else if (spin_status == COUNTER_CLOCKWISE)
                target = final_armors.at(0);
        }

        //判断装甲板是否切换，若切换将变量置1
        auto delta_t = src_timestamp - src_timestamp;
        auto delta_dist = (target.center3d_world - last_armor.center3d_world).norm();
        auto velocity = (delta_dist / delta_t) * 1e3;
        if ((target.id != last_armor.id || !last_armor.roi.contains((target.center2d))) &&
            is_last_target_exists)
            is_target_switched = true;
        else
            is_target_switched = false;
#ifdef USING_PREDICT
        if (is_target_switched)
        {
            predictor.initParam(predictor_param_loader);
            aiming_point = target.center3d_cam;
        }
        else
        {
            auto aiming_point_world = predictor.predict(target.center3d_world, src_timestamp);
            // aiming_point = aiming_point_world;
            aiming_point = coordsolver.worldToCam(aiming_point_world, rmat_imu);
        }
#else
    // aiming_point = coordsolver.worldToCam(target.center3d_world,rmat_imu);
    aiming_point = target.center3d_cam;
#endif //USING_PREDICT
    }
    ///----------------------------------常规击打---------------------------------------
    else
    {
        for (auto iter = ID_candiadates.first; iter != ID_candiadates.second; ++iter)
        {
            // final_armors.push_back((*iter).second.last_armor);
            final_trackers.push_back(&(*iter).second);
        }
        //进行目标选择
        auto tracker = chooseTargetTracker(final_trackers, src_timestamp);
        tracker->last_selected_timestamp = src_timestamp;
        tracker->selected_cnt++;
        target = tracker->last_armor;
        //判断装甲板是否切换，若切换将变量置1
        // auto delta_t = src_timestamp - last_timestamp;
        auto delta_dist = (target.center3d_world - last_armor.center3d_world).norm();
        auto velocity = (delta_dist / delta_t) * 1e3;
        // cout<<(delta_dist >= max_delta_dist)<<" "<<!last_armor.roi.contains(target.center2d)<<endl;
        if ((target.id != last_armor.id || !last_armor.roi.contains((target.center2d))) &&
            is_last_target_exists)
            is_target_switched = true;
        else
            is_target_switched = false;


#ifdef USING_PREDICT
        //目前类别预测不是十分稳定,若之后仍有问题，可以考虑去除类别判断条件
        if (is_target_switched)
        {
            predictor.initParam(predictor_param_loader);
            // cout<<"initing"<<endl;
            aiming_point = target.center3d_cam;
        }
        else
        {
            auto aiming_point_world = predictor.predict(target.center3d_world, src_timestamp);
            // aiming_point = aiming_point_world;
            aiming_point = coordsolver.worldToCam(aiming_point_world, rmat_imu);
        }
#else
    // aiming_point = coordsolver.worldToCam(target.center3d_world,rmat_imu);
    aiming_point = target.center3d_cam;
#endif //USING_PREDICT
    }
#ifdef ASSIST_LABEL
    auto label_name = path_prefix + to_string(src_timestamp) + ".txt";
    string content;

    int cls = 0;
    if (target.id == 7)
        cls = 9 * target.color - 1;
    if (target.id != 7)
        cls = target.id + target.color * 9;
    
    content.append(to_string(cls) + " ");
    for (auto apex : target.apex2d)
    {
        content.append(to_string((apex.x - roi_offset.x) / input_size.width));
        content.append(" ");
        content.append(to_string((apex.y - roi_offset.y) / input_size.height));
        content.append(" ");
    }
    content.pop_back();
    cout<<to_string(src_timestamp) + " "<<content<<endl;
    file.open(label_name,std::ofstream::app);
    file<<content;
    file.close();
    usleep(5000);
#endif //ASSIST_LABEL

    if (target.color == 2)
        dead_buffer_cnt++;
    else
        dead_buffer_cnt = 0;
    //获取装甲板中心与装甲板面积以下一次ROI截取使用
    last_roi_center = target.center2d;
    // last_roi_center = Point2i(512,640);
    last_armor = target;
    lost_cnt = 0;
    last_timestamp = src_timestamp;
    last_target_area = target.area;
    last_aiming_point = aiming_point;
    is_last_target_exists = true;
    last_armors.clear();
    last_armors = armors;
#ifdef SHOW_AIM_CROSS
    line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), {0,255,0}, 1);
    line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), {0,255,0}, 1);
#endif //SHOW_AIM_CROSS

#ifdef SHOW_ALL_ARMOR
    for (auto armor :armors)
    {
        putText(src.img, fmt::format("{:.2f}", armor.conf),armor.apex2d[3],FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);
        if (armor.color == 0)
            putText(src.img, fmt::format("B{}",armor.id),armor.apex2d[0],FONT_HERSHEY_SIMPLEX, 1, {255, 100, 0}, 2);
        if (armor.color == 1)
            putText(src.img, fmt::format("R{}",armor.id),armor.apex2d[0],FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255}, 2);
        if (armor.color == 2)
            putText(src.img, fmt::format("N{}",armor.id),armor.apex2d[0],FONT_HERSHEY_SIMPLEX, 1, {255, 255, 255}, 2);
        if (armor.color == 3)
            putText(src.img, fmt::format("P{}",armor.id),armor.apex2d[0],FONT_HERSHEY_SIMPLEX, 1, {255, 100, 255}, 2);
        for(int i = 0; i < 4; i++)
            line(src.img, armor.apex2d[i % 4], armor.apex2d[(i + 1) % 4], {0,255,0}, 1);
        rectangle(src.img, armor.roi, {255, 0, 255}, 1);
        auto armor_center = coordsolver.reproject(armor.center3d_cam);
        circle(src.img, armor_center, 4, {0, 0, 255}, 2);
    }
#endif //SHOW_ALL_ARMOR

#ifdef SHOW_PREDICT
    auto aiming_2d = coordsolver.reproject(aiming_point);
    circle(src.img, aiming_2d, 2, {0, 255, 255}, 2);
#endif //SHOW_PREDICT

    auto angle = coordsolver.getAngle(aiming_point, rmat_imu);
    //若预测出错则直接世界坐标系下坐标作为击打点
    if (isnan(angle[0]) || isnan(angle[1]))
        angle = coordsolver.getAngle(target.center3d_cam, rmat_imu);
    auto time_predict = std::chrono::steady_clock::now();

    double dr_crop_ms = std::chrono::duration<double,std::milli>(time_crop - time_start).count();
    double dr_infer_ms = std::chrono::duration<double,std::milli>(time_infer - time_crop).count();
    double dr_predict_ms = std::chrono::duration<double,std::milli>(time_predict - time_infer).count();
    double dr_full_ms = std::chrono::duration<double,std::milli>(time_predict - time_start).count();

#ifdef SHOW_FPS
    putText(src.img, fmt::format("FPS: {}", int(1000 / dr_full_ms)), {10, 25}, FONT_HERSHEY_SIMPLEX, 1, {0,255,0});
#endif //SHOW_FPS

#ifdef SHOW_IMG
    namedWindow("dst",0);
    imshow("dst",src.img);
    waitKey(1);
#endif //SHOW_IMG
#ifdef PRINT_LATENCY
    //降低输出频率，避免影响帧率
    if (src_timestamp % 10 == 0)
    {
        fmt::print(fmt::fg(fmt::color::gray), "-----------TIME------------\n");
        fmt::print(fmt::fg(fmt::color::blue_violet), "Crop: {} ms\n"   ,dr_crop_ms);
        fmt::print(fmt::fg(fmt::color::golden_rod), "Infer: {} ms\n",dr_infer_ms);
        fmt::print(fmt::fg(fmt::color::green_yellow), "Predict: {} ms\n",dr_predict_ms);
        fmt::print(fmt::fg(fmt::color::orange_red), "Total: {} ms\n",dr_full_ms);
    }
#endif //PRINT_LATENCY
    // cout<<target.center3d_world<<endl;
    // cout<<endl;
#ifdef PRINT_TARGET_INFO
    fmt::print(fmt::fg(fmt::color::gray), "-----------INFO------------\n");
    fmt::print(fmt::fg(fmt::color::blue_violet), "Yaw: {} \n",angle[0]);
    fmt::print(fmt::fg(fmt::color::golden_rod), "Pitch: {} \n",angle[1]);
    fmt::print(fmt::fg(fmt::color::green_yellow), "Dist: {} m\n",(float)target.center3d_cam.norm());
    fmt::print(fmt::fg(fmt::color::white), "Target: {} \n",target.key);
    fmt::print(fmt::fg(fmt::color::white), "Target Type: {} \n",target.type == SMALL ? "SMALL" : "BIG");
    fmt::print(fmt::fg(fmt::color::orange_red), "Is Spinning: {} \n",is_target_spinning);
    fmt::print(fmt::fg(fmt::color::orange_red), "Is Switched: {} \n",is_target_switched);
#endif //PRINT_TARGET_INFO
#ifdef SAVE_AUTOAIM_LOG
    LOG(INFO) <<"[AUTOAIM] LATENCY: "<< "Crop: " << dr_crop_ms << " ms" << " Infer: " << dr_infer_ms << " ms" << " Predict: " << dr_predict_ms << " ms" << " Total: " << dr_full_ms << " ms";
    LOG(INFO) <<"[AUTOAIM] TARGET_INFO: "<< "Yaw: " << angle[0] << " Pitch: " << angle[1] << " Dist: " << (float)target.center3d_cam.norm()
                    << " Target: " << target.key << " Is Spinning: " << is_target_spinning<< " Is Switched: " << is_target_switched;
    LOG(INFO) <<"[AUTOAIM] PREDICTED: "<<"X: "<<aiming_point[0]<<" Y: "<<aiming_point[1]<<" Z: " << aiming_point[2];
#endif //SAVE_AUTOAIM_LOG

    //若预测出错取消本次数据发送
    if (isnan(angle[0]) || isnan(angle[1]))
    {
        LOG(ERROR)<<"NAN Detected! Data Transmit Aborted!";
        return ;
    }

    // data = {(float)angle[1], (float)angle[0], (float)target.center3d_cam.norm(), is_target_switched, 1, is_target_spinning, 0};
    return ;
}

int main(int argc, char** argv)
{
    coordsolver.loadParam(camera_param_path,camera_name);
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "buff_predict"); // 初始化ROS节点
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("B_track_predict", 100, callback_predict);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_img = it.subscribe("images", 10, imageCallback);
    ros::spin();
    return 0;
}