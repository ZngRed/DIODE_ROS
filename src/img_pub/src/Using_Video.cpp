#include <Using_Video/Using_Video.h>

int main(int argc, char** argv)
{
    auto time_start=std::chrono::steady_clock::now();
    setlocale(LC_ALL,"");
    string path = "/home/dhu/Sample/test_armor_2.api";
    cv::VideoCapture cap(path);
    cv::Mat original_image;

    ros::init(argc, argv, "Using_Video"); // 初始化ROS节点
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("images", 1); // 创建图像发布者 话题名称为images
                                                                      // TODO:作为外参引入
    ros::Publisher pub = nh.advertise<std_msgs::Int64>("src_timestamp", 1);
    sensor_msgs::ImagePtr Omsg_img;
    std_msgs::Int64 Omsg_timestamp;

    ros::Rate loop_rate(50); // 发布频率为1Hz
    while (nh.ok()){
        auto time_cap=std::chrono::steady_clock::now();
        cap.read(original_image);
        Omsg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", original_image).toImageMsg(); // 转换为ROS图像消息
        Omsg_timestamp.data = (int)(std::chrono::duration<double,std::milli>(time_cap - time_start).count());
        image_pub.publish(Omsg_img); // 发布图像消息
        pub.publish(Omsg_timestamp);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}