#include <buff/buff_inference.h>

ros::Publisher pub;
rm_msgs::B_infer_fan Omsg_fan;
rm_msgs::B_infer_track Omsg_track;

buff_infer infer;

static constexpr int INPUT_W = 416;    // Width of input
static constexpr int INPUT_H = 416;    // Height of input
static constexpr int NUM_CLASSES = 2;  // Number of classes
static constexpr int NUM_COLORS = 2;   // Number of color
static constexpr int TOPK = 128;       // TopK
static constexpr float NMS_THRESH  = 0.1;
static constexpr float BBOX_CONF_THRESH = 0.6;
static constexpr float MERGE_CONF_ERROR = 0.15;
static constexpr float MERGE_MIN_IOU = 0.2;

static inline int argmax(const float *ptr, int len) 
{
    int max_arg = 0;
    for (int i = 1; i < len; i++) {
        if (ptr[i] > ptr[max_arg]) max_arg = i;
    }
    return max_arg;
}

inline cv::Mat scaledResize(cv::Mat& img, Eigen::Matrix<float,3,3> &transform_matrix)
{
    float r = std::min(INPUT_W / (img.cols * 1.0), INPUT_H / (img.rows * 1.0));
    int unpad_w = r * img.cols;
    int unpad_h = r * img.rows;
    
    int dw = INPUT_W - unpad_w;
    int dh = INPUT_H - unpad_h;

    dw /= 2;
    dh /= 2;
    
    transform_matrix << 1.0 / r, 0      , -dw / r,
                        0      , 1.0 / r, -dh / r,
                        0      , 0      , 1      ;
    
    Mat re;
    cv::resize(img, re, Size(unpad_w,unpad_h));
    Mat out;
    cv::copyMakeBorder(re, out, dh, dh, dw, dw, BORDER_CONSTANT);

    return out;
}

static void generate_grids_and_stride(const int target_w, const int target_h,
                                        std::vector<int>& strides, std::vector<GridAndStride>& grid_strides)
{
    for (auto stride : strides){
        int num_grid_w = target_w / stride;
        int num_grid_h = target_h / stride;
        for (int g1 = 0; g1 < num_grid_h; g1++){
            for (int g0 = 0; g0 < num_grid_w; g0++){
                grid_strides.push_back((GridAndStride){g0, g1, stride});
            }
        }
    }
}

static void generateYoloxProposals(std::vector<GridAndStride> grid_strides, const float* feat_ptr,
                                    Eigen::Matrix<float,3,3> &transform_matrix,float prob_threshold,
                                    std::vector<BuffObject>& objects)
{

    const int num_anchors = grid_strides.size();
    //Travel all the anchors
    for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++){
        const int grid0 = grid_strides[anchor_idx].grid0;
        const int grid1 = grid_strides[anchor_idx].grid1;
        const int stride = grid_strides[anchor_idx].stride;
	      const int basic_pos = anchor_idx * (11 + NUM_COLORS + NUM_CLASSES);

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
        float x_5 = (feat_ptr[basic_pos + 8] + grid0) * stride;
        float y_5 = (feat_ptr[basic_pos + 9] + grid1) * stride;

        int box_color = argmax(feat_ptr + basic_pos + 11, NUM_COLORS);
        int box_class = argmax(feat_ptr + basic_pos + 11 + NUM_COLORS, NUM_CLASSES);

        float box_objectness = (feat_ptr[basic_pos + 10]);
        
        float color_conf = (feat_ptr[basic_pos + 11 + box_color]);
        float cls_conf = (feat_ptr[basic_pos + 11 + NUM_COLORS + box_class]);

        // cout<<box_objectness<<endl;
        // float box_prob = (box_objectness + cls_conf + color_conf) / 3.0;
        float box_prob = box_objectness;

        if (box_prob >= prob_threshold){
            BuffObject obj;

            Eigen::Matrix<float,3,5> apex_norm;
            Eigen::Matrix<float,3,5> apex_dst;

            apex_norm << x_1,x_2,x_3,x_4,x_5,
                        y_1,y_2,y_3,y_4,y_5,
                        1,1,1,1,1;
            
            apex_dst = transform_matrix * apex_norm;

            for (int i = 0; i < 5; i++)
                obj.apex[i] = cv::Point2f(apex_dst(0,i),apex_dst(1,i));
            for (int i = 0; i < 5; i++){
                obj.apex[i] = cv::Point2f(apex_dst(0,i),apex_dst(1,i));
                obj.pts.push_back(obj.apex[i]);
            }
            vector<cv::Point2f> tmp(obj.apex,obj.apex + 5);
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
static inline float intersection_area(const BuffObject& a, const BuffObject& b)
{
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

static void qsort_descent_inplace(std::vector<BuffObject>& faceobjects, int left, int right)
{
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j){
        while (faceobjects[i].prob > p)
            i++;

        while (faceobjects[j].prob < p)
            j--;

        if (i <= j){
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

static void qsort_descent_inplace(std::vector<BuffObject>& objects)
{
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

static void nms_sorted_bboxes(std::vector<BuffObject>& faceobjects, std::vector<int>& picked,
                            float nms_threshold)
{
    picked.clear();
    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++){
        std::vector<cv::Point2f> object_apex_tmp(faceobjects[i].apex, faceobjects[i].apex + 5);
        areas[i] = contourArea(object_apex_tmp);
        // areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++){
        BuffObject& a = faceobjects[i];
        std::vector<cv::Point2f> apex_a(a.apex, a.apex + 5);
        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++){
            BuffObject& b = faceobjects[picked[j]];
            std::vector<cv::Point2f> apex_b(b.apex, b.apex + 5);
            std::vector<cv::Point2f> apex_inter;
            // intersection over union
            // float inter_area = intersection_area(a, b);
            // float union_area = areas[i] + areas[picked[j]] - inter_area;
            //TODO:此处耗时较长，大约1ms，可以尝试使用其他方法计算IOU与多边形面积
            float inter_area = intersectConvexConvex(apex_a,apex_b,apex_inter);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            float iou = inter_area / union_area;

            if (iou > nms_threshold || isnan(iou)){
                keep = 0;
                //Stored for Merge
                if (iou > MERGE_MIN_IOU && abs(a.prob - b.prob) < MERGE_CONF_ERROR 
                                        && a.cls == b.cls && a.color == b.color){
                    for (int i = 0; i < 5; i++){
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

static void decodeOutputs(const float* prob, std::vector<BuffObject>& objects,
                            Eigen::Matrix<float,3,3> &transform_matrix, const int img_w, const int img_h)
{
        std::vector<BuffObject> proposals;
        std::vector<int> strides = {8, 16, 32};
        std::vector<GridAndStride> grid_strides;

        generate_grids_and_stride(INPUT_W, INPUT_H, strides, grid_strides);
        generateYoloxProposals(grid_strides, prob, transform_matrix, BBOX_CONF_THRESH, proposals);
        qsort_descent_inplace(proposals);

        if (proposals.size() >= TOPK) proposals.resize(TOPK);
        std::vector<int> picked;
        nms_sorted_bboxes(proposals, picked, NMS_THRESH);
        int count = picked.size();
        objects.resize(count);

        for (int i = 0; i < count; i++){
            objects[i] = proposals[picked[i]];
        }
}

void drawPred(Mat& frame, std::vector<Point2f> landmark)   // Draw the predicted bounding box
{
    // cout<<"KKK"<<endl;
    // circle(frame, Point(0, 0), 5, Scalar(255, 255, 0), -1);
    // circle(frame, Point(416, 416), 5, Scalar(255, 255, 0), -1);
    //画出扇叶五点、扇叶中心、能量机关中心
    // cout<<landmark.size()<<endl;
    for (int i = 0; i < 5; i++)
    {
      circle(frame, Point(landmark[i].x, landmark[i].y), 5, Scalar(0, 255, 0), -1);
    }
}

buff_infer::buff_infer()
{
    model_init();
    is_last_target_exists = false;
    lost_cnt = 0;
    input_size = {640, 640};
    last_bullet_speed = 0;
}
buff_infer::~buff_infer()
{
}
/**
 * @brief 初始化模型
*/
void buff_infer::model_init()
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
}

/**
 * @brief 模型推理
 * 
 * @param img 原始图像
*/
void buff_infer::infer(Mat &img, std::vector<Point2f> &points)
{
    ros::NodeHandle nh;

    cv::Mat pr_img = scaledResize(img,transfrom_matrix);
    cv::Mat pre, pre_split[3];
    pr_img.convertTo(pre,CV_32F);
    cv::split(pre,pre_split);
    vector<BuffObject> objects;

    Blob::Ptr imgBlob = infer_request.GetBlob(input_name);
    InferenceEngine::MemoryBlob::Ptr mblob = InferenceEngine::as<InferenceEngine::MemoryBlob>(imgBlob);

    auto mblobHolder = mblob->wmap();
    float *blob_data = mblobHolder.as<float *>();

    auto img_offset = INPUT_W * INPUT_H;
    
    //Copy img into blob
    for(int c = 0;c < 3;c++){
       memcpy(blob_data, pre_split[c].data, INPUT_W * INPUT_H * sizeof(float));
       blob_data += img_offset;
    }

    // Do inference
    infer_request.Infer();

    auto moutputHolder = moutput->rmap();
    const float* net_pred = moutputHolder.as<const PrecisionTrait<Precision::FP32>::value_type*>();
    int img_w = img.cols;
    int img_h = img.rows;
    decodeOutputs(net_pred, objects, transfrom_matrix, img_w, img_h);
    for (auto object = objects.begin(); object != objects.end(); ++object){
        if ((*object).pts.size() >= 10){
            auto N = (*object).pts.size();
            cv::Point2f pts_final[5];
            std::vector<Point2f> landmark;
            for (int i = 0; i < N; i++){
                pts_final[i % 5]+=(*object).pts[i];
            }
            for (int i = 0; i < 5; i++){
                pts_final[i].x = pts_final[i].x / (N / 5);
                pts_final[i].y = pts_final[i].y / (N / 5);
                landmark.push_back(pts_final[i]);
                points.push_back(pts_final[i]);
            }
            drawPred(img, landmark);

            Omsg_fan.apex_0.x = (*object).apex[0].x;
            Omsg_fan.apex_0.y = (*object).apex[0].y;
            Omsg_fan.apex_1.x = (*object).apex[1].x;
            Omsg_fan.apex_1.y = (*object).apex[1].y;
            Omsg_fan.apex_2.x = (*object).apex[2].x;
            Omsg_fan.apex_2.y = (*object).apex[2].y;
            Omsg_fan.apex_3.x = (*object).apex[3].x;
            Omsg_fan.apex_3.y = (*object).apex[3].y;
            Omsg_fan.apex_4.x = (*object).apex[4].x;
            Omsg_fan.apex_4.y = (*object).apex[4].y;
            Omsg_fan.cls = (*object).cls;
            Omsg_fan.color = (*object).color;
            Omsg_fan.prob = (*object).prob;
            pub = nh.advertise<rm_msgs::B_infer_fan>("B_infer_fan", 10);
            pub.publish(Omsg_fan);
        }
    }
    Omsg_track.is_last_target_exists = is_last_target_exists;
    Omsg_track.lost_cnt = lost_cnt;
    Omsg_track.last_timestamp = last_timestamp;
    Omsg_track.last_target_area = last_target_area;
    Omsg_track.last_bullet_speed = last_bullet_speed;
    Omsg_track.last_roi_center.x = last_roi_center.x;
    Omsg_track.last_roi_center.y = last_roi_center.y;
    Omsg_track.roi_offset.x = roi_offset.x;
    Omsg_track.roi_offset.y = roi_offset.y;
    Omsg_track.input_size = static_cast<int>(input_size.width);
    pub = nh.advertise<rm_msgs::B_infer_track>("B_infer_track", 10);
    pub.publish(Omsg_track);

    return ;
}

/**
 * @brief 根据上次装甲板位置截取ROI
 * 
 * @param img 所需处理的图像
 * @return ** Point2i ROI中心点
 */
Point2i buff_infer::cropImageByROI(Mat &img)
{
    if (!is_last_target_exists)
    {
        //当丢失目标帧数过多或lost_cnt为初值
        if (lost_cnt > max_lost_cnt || lost_cnt == 0)
        {
            return Point2i(0,0);
        }
    }
    //若目标大小大于阈值
    // cout<<last_target_area / img.size().area()<<endl;
    if ((last_target_area / img.size().area()) > no_crop_thres)
    {
        return Point2i(0,0);
    }
    //处理X越界
    if (last_roi_center.x <= input_size.width / 2)
        last_roi_center.x = input_size.width / 2;
    else if (last_roi_center.x > (img.size().width - input_size.width / 2))
        last_roi_center.x = img.size().width - input_size.width / 2;
    //处理Y越界
    if (last_roi_center.y <= input_size.height / 2)
        last_roi_center.y = input_size.height / 2;
    else if (last_roi_center.y > (img.size().height - input_size.height / 2))
        last_roi_center.y = img.size().height - input_size.height / 2;

    //左上角顶点
    auto offset = last_roi_center - Point2i(input_size.width / 2, input_size.height / 2);
    Rect roi_rect = Rect(offset, input_size);
    img(roi_rect).copyTo(img);

    return offset;
}

void imageCallback(const sensor_msgs::ImageConstPtr& Imsg)
{
    cv::Mat img = cv_bridge::toCvShare(Imsg, "bgr8")->image;
    infer.roi_offset = infer.cropImageByROI(img);
    std::vector<Point2f> points;
    infer.infer(img, points);
    cv::imshow("IMG", img);
    cv::waitKey(1);
    return ;
}

void callback_timestamp(const std_msgs::Int8 &Imsg)
{
    infer.last_timestamp = Imsg.data;
    return ;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    cv::namedWindow("IMG");
    ros::init(argc, argv, "buff_inference"); // 初始化ROS节点
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Subscriber sub_timestamp = nh.subscribe("src_timestamp", 10, callback_timestamp);
    image_transport::Subscriber sub_img = it.subscribe("images", 10, imageCallback);
    ros::spin();
    return 0;
}