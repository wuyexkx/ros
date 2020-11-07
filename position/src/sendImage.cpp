#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_images");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/new_camera/left_image", 1);

    cv::Mat image = cv::imread("/home/wuyexkx/catkin_ws/src/position/images/jg-000.jpg", CV_LOAD_IMAGE_COLOR);
    if(image.empty()) printf("\nopen image error!\n");
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();//图像格式转换
    ros::Rate loop_rate(10);//每秒1帧

    char* s[] = {"/home/wuyexkx/catkin_ws/src/position/images/jg-000.jpg",
                "/home/wuyexkx/catkin_ws/src/position/images/mb-001.jpg", 
                "/home/wuyexkx/catkin_ws/src/position/images/jg-002.jpg", 
                "/home/wuyexkx/catkin_ws/src/position/images/jg-003.jpg",
                "/home/wuyexkx/catkin_ws/src/position/images/jg-004.jpg",
                "/home/wuyexkx/catkin_ws/src/position/images/mb-003.jpg",
                "/home/wuyexkx/catkin_ws/src/position/images/jg-005.jpg",
                "/home/wuyexkx/catkin_ws/src/position/images/jg-006.jpg",
                "/home/wuyexkx/catkin_ws/src/position/images/jg-007.jpg",
                "/home/wuyexkx/catkin_ws/src/position/images/mb-002.jpg"};
    int i = 0;
    while (nh.ok()) 
    {
        cv::Mat image = cv::imread(s[i], CV_LOAD_IMAGE_COLOR);
        if(image.empty()) printf("\nopen image error!\n");
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

        pub.publish(msg);
        // ros::spinOnce();
        ROS_INFO("Send a image OK!");
        loop_rate.sleep();
        i++;
        if(i==9) i = 0;
    }

}
