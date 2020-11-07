#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "my_function.h"
#include "sys/time.h"
#include <set>

using namespace DBoW3;
using namespace std;


void imageCallback(const sensor_msgs::ImageConstPtr& msg, const vector<string>& names, Database *db, QueryResults& ret, std_msgs::String& build_name)
{
    try
    {   
        // struct timeval startTime, extractTime, endTime;
        // gettimeofday(&startTime, NULL);  // 时间

        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        if((image).empty())throw std::runtime_error("Read the image is empty!");
        // 转灰度
        cv::Mat grayImage;
        cv::cvtColor(image, grayImage, CV_BGR2GRAY);
        cv::Mat feature = load_Feature(grayImage);
        // gettimeofday(&extractTime, NULL); // 时间
        
        int rst_nums = 7;
        (*db).query(feature, ret, rst_nums);
        // gettimeofday(&endTime, NULL); // 时间

        // 处理搜索结果, 得到 前rst_nums个结果中包含最多的那个的 数量和其迭代器
        // 如: aa bb bb aa bb ee bb 重复最多的是bb, 次数为4
        //    将切取的名称存入rst_set
        static std::multiset<string> rst_set;
        for(int i=0; i<rst_nums; ++i)
        {
            string maxvlupath = names[ret[i].Id];
            rst_set.insert(maxvlupath.substr(maxvlupath.rfind("/")+1, 
                          (maxvlupath.rfind("-")-(maxvlupath.rfind("/")+1))));  
        }       
        //    得到重复最多的元素的迭代器 和 对应次数
        auto temp = rst_set.begin(); // 迭代器
        int cnt = 0;                 // 计数器
        for(auto it=rst_set.begin(); it!=rst_set.end(); ++it)
        {
            if(cnt <= rst_set.count(*it))
            {
                cnt = rst_set.count(*it);
                temp = it;
            }
        }

        // 首个检索结果要满足score, 并且重复次数超过3才算检索到正确结果
        // ***************************************************
        if(ret[0].Score > 0.64 && cnt > 3) 
        {
            ROS_INFO(" ");
            cout << "=========== Current position: " + *temp + " ============" << endl; // 带时间戳 显示检索结果,
            for(int i=0;i<rst_nums; i++) // 一次输出前7个结果所对应的图片
            {
                std::cout << to_string(i)+": "+to_string(ret[i].Score)+"--"+names[ret[i].Id] << std::endl;
            }
            build_name.data = *temp; // 保存结果
        }
        else // 否则没有检索到
        {
            build_name.data = "unknow";
        }
        rst_set.clear(); // 一次回调完成清空set
       
        // 计算检索一张图所需时间及帧率
        // 时间： 总时间=特征提取时间+检索时间
        // 帧率： 1秒可以有多少张图片 检索当前数据库
        // double a = (double)(endTime.tv_usec-startTime.tv_usec)/1000000;
        // double b = (double)(extractTime.tv_usec-startTime.tv_usec)/1000000;
        // double c = (double)(endTime.tv_usec-extractTime.tv_usec)/1000000;
        // ROS_INFO(("totalTime   :" + to_string(a) +"s FPS(1000):" + to_string(1/a)).c_str());
        // ROS_INFO(("extractTime :" + to_string(b) +"s FPS(1000):" + to_string(1/b)).c_str());
        // ROS_INFO(("retrieveTime:" + to_string(c) +"s FPS(1000):" + to_string(1/c)).c_str());
    }    
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from %s to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char** argv)
{
    // 初始化ros
    ros::init(argc, argv, "node_position");
    ros::NodeHandle nh;

    vector<string> names; // 存放数据库图片的名称
    QueryResults ret;
// ========================================================================
    string database_path = "/home/wuyexkx/catkin_ws/src/position/database";
    loadTxt(database_path + "/mould_names.txt", names);
    Database db(database_path + "/mould_db.yml.gz");
// ========================================================================

    // 初始化订阅器
    std_msgs::String build_name; // 需要发布的数据
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/new_camera/left_image", 1, 
                    boost::bind(imageCallback, _1, names, &db, ret, build_name));
    
	//初始化发布器
    ros::Publisher build_pub = 
        nh.advertise<std_msgs::String>("position_name", 10); // 缓冲区中在大于10个的时候就会开始丢弃先前的数据
	//定义发布频率
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        // 发布消息
        build_pub.publish(build_name);
        // 通过 spinOnce 调用一次订阅器的回调函数(也就是接收一次消息)
        ros::spinOnce();
        // 阻塞此线程,同步循环频率到设定频率
        loop_rate.sleep();

    }
}
