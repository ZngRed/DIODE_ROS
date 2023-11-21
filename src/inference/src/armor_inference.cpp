#include <armor/armor_inference.h>

static constexpr int INPUT_W = 416;    // Width of input
static constexpr int INPUT_H = 416;    // Height of input
static constexpr int NUM_CLASSES = 8;  // Number of classes
static constexpr int NUM_COLORS = 4;   // Number of color
static constexpr int TOPK = 128;       // TopK
static constexpr float NMS_THRESH = 0.3;
static constexpr float BBOX_CONF_THRESH = 0.75;
static constexpr float MERGE_CONF_ERROR = 0.15;
static constexpr float MERGE_MIN_IOU = 0.9;

static inline int argmax(const float *ptr, int len) 
{
    int max_arg = 0;
    for (int i = 1; i < len; i++) {
        if (ptr[i] > ptr[max_arg]) max_arg = i;
    }
    return max_arg;
}

/**
 * @brief Resize the image using letterbox
 * @param img Image before resize
 * @param transform_matrix Transform Matrix of Resize
 * @return Image after resize
 */
inline cv::Mat scaledResize(cv::Mat& img, Eigen::Matrix<float,3,3> &transform_matrix)
{
    float r = std::min(INPUT_W / (img.cols * 1.0), INPUT_H / (img.rows * 1.0));
    int unpad_w = r * img.cols;
    int unpad_h = r * img.rows;
    
    int dw = INPUT_W - unpad_w;
    int dh = INPUT_H - unpad_h;

    dw /= 2;
    dh /= 2;
    
    transform_matrix << 1.0 / r, 0, -dw / r,
                        0, 1.0 / r, -dh / r,
                        0, 0, 1;
    
    Mat re;
    cv::resize(img, re, Size(unpad_w,unpad_h));
    Mat out;
    cv::copyMakeBorder(re, out, dh, dh, dw, dw, BORDER_CONSTANT);

    return out;
}

/**
 * @brief Generate grids and stride.
 * @param target_w Width of input.
 * @param target_h Height of input.
 * @param strides A vector of stride.
 * @param grid_strides Grid stride generated in this function.
 */
static void generate_grids_and_stride(const int target_w, const int target_h,
                                        std::vector<int>& strides, std::vector<GridAndStride>& grid_strides)
{
    for (auto stride : strides)
    {
        int num_grid_w = target_w / stride;
        int num_grid_h = target_h / stride;

        for (int g1 = 0; g1 < num_grid_h; g1++)
        {
            for (int g0 = 0; g0 < num_grid_w; g0++)
            {
                grid_strides.push_back((GridAndStride){g0, g1, stride});
            }
        }
    }
}

/**
 * @brief Generate Proposal
 * @param grid_strides Grid strides
 * @param feat_ptr Original predition result.
 * @param prob_threshold Confidence Threshold.
 * @param objects Objects proposed.
 */
static void generateYoloxProposals(std::vector<GridAndStride> grid_strides, const float* feat_ptr,
                                    Eigen::Matrix<float,3,3> &transform_matrix,float prob_threshold,
                                    std::vector<ArmorObject>& objects)
{

    const int num_anchors = grid_strides.size();
    //Travel all the anchors
    for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++)
    {
        const int grid0 = grid_strides[anchor_idx].grid0;
        const int grid1 = grid_strides[anchor_idx].grid1;
        const int stride = grid_strides[anchor_idx].stride;

	    const int basic_pos = anchor_idx * (9 + NUM_COLORS + NUM_CLASSES);

        // yolox/models/yolo_head.py decode logic
        //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
        //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
        float x_1 = (feat_ptr[basic_pos + 0] + grid0) * stride;
        float y_1 = (feat_ptr[basic_pos + 1] + grid1) * stride;
        float x_2 = (feat_ptr[basic_pos + 2] + grid0) * stride;
        float y_2 = (feat_ptr[basic_pos + 3] + grid1) * stride;
        float x_3 = (feat_ptr[basic_pos + 4] + grid0) * stride;
        float y_3 = (feat_ptr[basic_pos + 5] + grid1) * stride;
        float x_4 = (feat_ptr[basic_pos + 6] + grid0) * stride;
        float y_4 = (feat_ptr[basic_pos + 7] + grid1) * stride;

        int box_color = argmax(feat_ptr + basic_pos + 9, NUM_COLORS);
        int box_class = argmax(feat_ptr + basic_pos + 9 + NUM_COLORS, NUM_CLASSES);

        float box_objectness = (feat_ptr[basic_pos + 8]);
        
        float color_conf = (feat_ptr[basic_pos + 9 + box_color]);
        float cls_conf = (feat_ptr[basic_pos + 9 + NUM_COLORS + box_class]);

        // float box_prob = (box_objectness + cls_conf + color_conf) / 3.0;
        float box_prob = box_objectness;

        if (box_prob >= prob_threshold)
        {
            ArmorObject obj;

            Eigen::Matrix<float,3,4> apex_norm;
            Eigen::Matrix<float,3,4> apex_dst;

            apex_norm << x_1,x_2,x_3,x_4,
                        y_1,y_2,y_3,y_4,
                        1,1,1,1;
            
            apex_dst = transform_matrix * apex_norm;

            for (int i = 0; i < 4; i++)
            {
                obj.apex[i] = cv::Point2f(apex_dst(0,i),apex_dst(1,i));
                obj.pts.push_back(obj.apex[i]);
            }
            
            vector<cv::Point2f> tmp(obj.apex,obj.apex + 4);
            obj.rect = cv::boundingRect(tmp);

            obj.cls = box_class;
            obj.color = box_color;
            obj.prob = box_prob;

            objects.push_back(obj);
        }

    } // point anchor loop
}

/**
 * @brief Calculate intersection area between two objects.
 * @param a Object a.
 * @param b Object b.
 * @return Area of intersection.
 */
static inline float intersection_area(const ArmorObject& a, const ArmorObject& b)
{
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

static void qsort_descent_inplace(std::vector<ArmorObject>& faceobjects, int left, int right)
{
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j)
    {
        while (faceobjects[i].prob > p)
            i++;

        while (faceobjects[j].prob < p)
            j--;

        if (i <= j)
        {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            if (left < j) qsort_descent_inplace(faceobjects, left, j);
        }
        #pragma omp section
        {
            if (i < right) qsort_descent_inplace(faceobjects, i, right);
        }
    }
}


static void qsort_descent_inplace(std::vector<ArmorObject>& objects)
{
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}


static void nms_sorted_bboxes(std::vector<ArmorObject>& faceobjects, std::vector<int>& picked,
                            float nms_threshold)
{
    picked.clear();

    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++)
    {
        areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++)
    {
        ArmorObject& a = faceobjects[i];

        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++)
        {
            ArmorObject& b = faceobjects[picked[j]];

            // intersection over union
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            float iou = inter_area / union_area;
            if (iou > nms_threshold || isnan(iou))
            {
                keep = 0;
                //Stored for Merge
                if (iou > MERGE_MIN_IOU && abs(a.prob - b.prob) < MERGE_CONF_ERROR 
                                        && a.cls == b.cls && a.color == b.color)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        b.pts.push_back(a.apex[i]);
                    }
                }
                // cout<<b.pts_x.size()<<endl;
            }
        }

        if (keep)
            picked.push_back(i);
    }
}

/**
 * @brief Decode outputs.
 * @param prob Original predition output.
 * @param objects Vector of objects predicted.
 * @param img_w Width of Image.
 * @param img_h Height of Image.
 */
static void decodeOutputs(const float* prob, std::vector<ArmorObject>& objects,
                            Eigen::Matrix<float,3,3> &transform_matrix, const int img_w, const int img_h)
{
        std::vector<ArmorObject> proposals;
        std::vector<int> strides = {8, 16, 32};
        std::vector<GridAndStride> grid_strides;

        generate_grids_and_stride(INPUT_W, INPUT_H, strides, grid_strides);
        generateYoloxProposals(grid_strides, prob, transform_matrix, BBOX_CONF_THRESH, proposals);
        qsort_descent_inplace(proposals);

        if (proposals.size() >= TOPK) 
            proposals.resize(TOPK);
        std::vector<int> picked;
        nms_sorted_bboxes(proposals, picked, NMS_THRESH);
        int count = picked.size();
        objects.resize(count);

        for (int i = 0; i < count; i++)
        {
            objects[i] = proposals[picked[i]];
        }
}

armor_infer::armor_infer()
{
    detector.model_init(network_path);
    lost_cnt = 0;
    dead_buffer_cnt = 0;
    is_last_target_exists = false;
    is_target_switched = false;
    last_target_area = 0;
    last_bullet_speed = 0;
    input_size = {416, 416};
    // predictor.initParam(predictor_param_loader);

#ifdef DETECT_RED
    detect_color = RED;
#endif //DETECT_RED
#ifdef DETECT_BLUE
    detect_color = BLUE;
#endif //DETECT_BLUE
}
armor_infer::~armor_infer()
{
}

void armor_infer::model_init()
{
    std::string input_model = "/home/dhu/Model/buff_0724.xml";
    // Initialize inference engine core
    ie.SetConfig({{CONFIG_KEY(CACHE_DIR), "../.cache"}});
    ie.SetConfig({{CONFIG_KEY(GPU_THROUGHPUT_STREAMS),"1"}});

    // Read a model in OpenVINO Intermediate Representation (.xml and .bin files) or ONNX (.onnx file) format
    network = ie.ReadNetwork(input_model);

    // Prepare input blobs
    InputInfo::Ptr input_info = network.getInputsInfo().begin()->second;
    input_name = network.getInputsInfo().begin()->first;

    // Prepare output blobs
    DataPtr output_info = network.getOutputsInfo().begin()->second;
    output_name = network.getOutputsInfo().begin()->first;

    // Loading a model to the device
    executable_network = ie.LoadNetwork(network, "GPU");

    // Create an infer request
    infer_request = executable_network.CreateInferRequest();

    // Process output
    const Blob::Ptr output_blob = infer_request.GetBlob(output_name);
    moutput = as<MemoryBlob>(output_blob);

    return ;
}

bool armor_infer::infer(Mat &src,std::vector<ArmorObject>& objects)
{
    if (src.empty())
    {
        fmt::print(fmt::fg(fmt::color::red), "[DETECT] ERROR: 传入了空的src\n");
        return false;
    }
    cv::Mat pr_img = scaledResize(src,transfrom_matrix);
    cv::Mat pre, pre_split[3];
    pr_img.convertTo(pre,CV_32F);
    cv::split(pre,pre_split);

    Blob::Ptr imgBlob = infer_request.GetBlob(input_name);     // just wrap Mat data by Blob::Ptr
    InferenceEngine::MemoryBlob::Ptr mblob = InferenceEngine::as<InferenceEngine::MemoryBlob>(imgBlob);
    // locked memory holder should be alive all time while access to its buffer happens
    auto mblobHolder = mblob->wmap();
    float *blob_data = mblobHolder.as<float *>();
    auto img_offset = INPUT_W * INPUT_H;
    //Copy img into blob
    for(int c = 0;c < 3;c++)
    {
        memcpy(blob_data, pre_split[c].data, INPUT_W * INPUT_H * sizeof(float));
        blob_data += img_offset;
    }
    infer_request.Infer();
    // Process output
    auto moutputHolder = moutput->rmap();
    const float* net_pred = moutputHolder.as<const PrecisionTrait<Precision::FP32>::value_type*>();
    int img_w = src.cols;
    int img_h = src.rows;
    decodeOutputs(net_pred, objects, transfrom_matrix, img_w, img_h);
    for (auto object = objects.begin(); object != objects.end(); ++object)
    {
        //对候选框预测角点进行平均,降低误差
        if ((*object).pts.size() >= 8)
        {
            auto N = (*object).pts.size();
            cv::Point2f pts_final[4];

            for (int i = 0; i < N; i++)
            {
                pts_final[i % 4]+=(*object).pts[i];
            }

            for (int i = 0; i < 4; i++)
            {
                pts_final[i].x = pts_final[i].x / (N / 4);
                pts_final[i].y = pts_final[i].y / (N / 4);
            }

            (*object).apex[0] = pts_final[0];
            (*object).apex[1] = pts_final[1];
            (*object).apex[2] = pts_final[2];
            (*object).apex[3] = pts_final[3];
        }
        (*object).area = (int)(calcTetragonArea((*object).apex));
    }
    if (objects.size() != 0)
        return true;
    else return false;
}

void imageCallback(const sensor_msgs::ImageConstPtr& Imsg)
{
    auto time_start=std::chrono::steady_clock::now();
    vector<ArmorObject> objects;
    vector<Armor> armors;

    auto input = src.img.clone();
    // cout<<input.size<<endl;
    //若为前哨站吊射模式,直接截取图像中间部分进行处理

#ifdef USING_IMU
    Eigen::Matrix3d rmat_imu = src.quat.toRotationMatrix();
    auto vec = rotationMatrixToEulerAngles(rmat_imu);
    // cout<<"Euler : "<<vec[0] * 180.f / CV_PI<<" "<<vec[1] * 180.f / CV_PI<<" "<<vec[2] * 180.f / CV_PI<<endl;
#else
    Eigen::Matrix3d rmat_imu = Eigen::Matrix3d::Identity();
#endif //USING_IMU

#ifndef DEBUG_WITHOUT_COM
    //设置弹速,若弹速大于10m/s值,且弹速变化大于0.5m/s则更新
    if (src.bullet_speed > 10)
    {
        double bullet_speed;
        if (abs(src.bullet_speed - last_bullet_speed) < 0.5 || abs(src.bullet_speed - last_bullet_speed) > 1.5)
        {
            bullet_speed = src.bullet_speed;
            predictor.setBulletSpeed(bullet_speed);
            coordsolver.setBulletSpeed(bullet_speed);
            last_bullet_speed = bullet_speed;
            LOG(INFO)<<"SPD Updated:"<<src.bullet_speed<<" : "<<last_bullet_speed;
        }
        
    }
#endif //DEBUG_WITHOUT_COM
    // cout<<"lost:"<<lost_cnt<<endl;

#ifdef USING_ROI
    //吊射模式采用固定ROI
    if (src.mode == 2)
    {
        // cout<<zoom_offset.x<<endl;
        //若不存在目标则进行中间区域ROI，默认大小为416x416
        if (lost_cnt >= max_lost_cnt && !is_last_target_exists)
        {
            roi_offset = zoom_offset;
            // cout<<
            input(Range(zoom_offset.y, zoom_offset.y + input_size.height),
                    Range(zoom_offset.x, zoom_offset.x + input_size.width)).copyTo(input);
        }
        else
        {
            roi_offset = cropImageByROI(input);
        }
    }
    else
    {
        roi_offset = cropImageByROI(input);
    }
#endif  //USING_ROI
    auto time_crop=std::chrono::steady_clock::now();
    //若未检测到目标
    if (!detector.detect())
    {
#ifdef SHOW_AIM_CROSS
        line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), {0,255,0}, 1);
        line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), {0,255,0}, 1);
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
        data = {(float)0, (float)0, (float)0, 0, 0, 0, 1};
        LOG(WARNING) <<"[AUTOAIM] No target detected!";
        return false;
    }
#ifdef ASSIST_LABEL
    auto img_name = path_prefix + to_string(src.timestamp) + ".jpg";
    imwrite(img_name,input);
#endif //ASSIST_LABEL
    auto time_infer = std::chrono::steady_clock::now();
    
    cv::Mat img = cv_bridge::toCvShare(Imsg, "bgr8")->image;
    roi_offset = cropImageByROI(img);
    cout<<roi_offset<<endl;
    infer.infer(img);
    cv::imshow("IMG", img);
    cv::waitKey(1);
    return ;
}

void callback_timestamp(const std_msgs::Int64::ConstPtr &Imsg)
{
    src_timestamp = Imsg->data;
    // cout<<"TIMESTAMP:  "<<last_timestamp<<endl;
    return ;
}

void callback_B_update(const rm_msgs::B_update::ConstPtr& Imsg)
{
    last_roi_center.x = Imsg->last_roi_center.x;
    last_roi_center.y = Imsg->last_roi_center.y;
    is_last_target_exists = Imsg->is_last_target_exists;
    return ;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    cv::namedWindow("IMG");
    ros::init(argc, argv, "armor_inference"); // 初始化ROS节点
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    image_transport::ImageTransport it(nh);
    ros::Subscriber sub_update = nh.subscribe("A_update", 10, callback_B_update);
    ros::Subscriber sub_timestamp = nh.subscribe("src_timestamp", 10, callback_timestamp);
    image_transport::Subscriber sub_img = it.subscribe("images", 10, imageCallback);
    ros::spin();
    return 0;
}