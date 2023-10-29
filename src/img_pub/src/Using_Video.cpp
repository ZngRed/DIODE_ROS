#include <Using_Video/Using_Video.h>

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    string path = "/home/dhu/Sample/Test.mp4";
    cv::VideoCapture cap(path);

    ros::init(argc, argv, "Using_Video"); // 初始化ROS节点
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("images", 10); // 创建图像发布者 话题名称为images
                                                                      // TODO:作为外参引入

    ros::Rate loop_rate(100); // 发布频率为100Hz
    while (nh.ok())
    {
        cv::Mat original_image;
        cap.read(original_image);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", original_image).toImageMsg(); // 转换为ROS图像消息
        image_pub.publish(msg); // 发布图像消息
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}