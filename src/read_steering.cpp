#include <string>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <queue>
#include <ctime>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Float64.h> 
#include <std_msgs/Float64MultiArray.h>
#include <mutex>
#include <thread>
#include <vector>
#include <pwd.h>
#include "vpm_msgs/ControlOrder.h"
#include "vpm_msgs/ControlOrders.h"
#include "matplotlibcpp.h"


using namespace std;
using namespace message_filters;


// double或float有效位数
std::string roundNum(double r, int precision)
{
    std::stringstream buff;
    buff << setprecision(precision) << r;
    return buff.str();
}

// Car_info按照时间戳保存控制 和 CAN读回的数据 保存到txt_path路径下
//  包括了：
//      两个回调函数，分别用于订阅CAN节点输出和控制节点输出 
//      打印CAN读回的信息
//      plot显示CAN和控制两路转向数据，包括一阶差分数据
class Car_info
{
public:
    Car_info(const std::string& txt_path);
    ~Car_info();
    inline void can_call_back(const std_msgs::Float64MultiArray::ConstPtr& car_info_msg);
    inline void ctrl_call_back(const vpm_msgs::ControlOrders::ConstPtr & ctrl_infos);
    inline void print_info();                 // 打印接收到的数据
    inline void show();                       // 显示缓冲大小(temp_size个点)

private:
    std::ofstream rdCANfile;                  // 两个文件
    std::ofstream rdCTRLfile;
    std::vector<double> rd_CAN_info;          // 全局存储读回的can值
    std::vector<vector<double>> rd_CTRL_info; // 4 x 3 全局据存储读到的ctrl值
    std::mutex cb_mutex;                      // 线程锁
    int can_cb_flag;                          // can调用回调函数的标志
    int ctrl_cb_flag;                         // ctrl调用回调函数的标志
};
Car_info::Car_info(const std::string& txt_path)
: rd_CAN_info(10, 0), can_cb_flag(0), ctrl_cb_flag(0), rd_CTRL_info(4,vector<double>(3, 1000))
{ 
    // 文件加上时间戳
    time_t now = time(0);
    tm *ltm = localtime(&now);
    // 输出 tm 结构的各个组成部分
    string data = to_string(1900 + ltm->tm_year) + "_" + 
                    to_string(1 + ltm->tm_mon) + "_" +
                    to_string(ltm->tm_mday) + "_" + 
                    to_string(ltm->tm_hour) + "_" + 
                    to_string(ltm->tm_min) + "_" + 
                    to_string(ltm->tm_sec);
    rdCANfile.open(txt_path + data + "_CAN_info.txt", ios::trunc);  //打开文件，清空
    rdCTRLfile.open(txt_path + data + "_CTRL_info.txt", ios::trunc);

    ROS_INFO(("\n=========== save txt to " + txt_path).c_str());
}
Car_info::~Car_info() 
{ 
    rdCANfile.close(); 
    rdCTRLfile.close(); 
}
void Car_info::can_call_back(const std_msgs::Float64MultiArray::ConstPtr& car_info_msg)
{
unsigned len = car_info_msg->data.size();	// 取得消息数组的长度 (6)
    // 0刹车状态:      1是踩下, 0是未踩下
    // 1档位状态:      0是P档, 1是R档, 2是N档, 3是D档, 6是B档
    // 2方向盘角度:     float 度
    // 3方向盘转角方向: 0是左, 1是右 (方向盘处于左边还是右边)
    // 4方向盘转速方向: 0是左, 1是右 (方向盘移动方向是左还是右)
    // 5方向盘转速:     度/秒

    cb_mutex.lock(); 
    // 标志置1,在打印的函数里可以显示了
    can_cb_flag = 1;
    // 暂存接收缓冲数据
    for(int i = 0; i < len; ++i) rd_CAN_info[i] = car_info_msg->data.at(i);

    // // 将数据保存为txt 多行为一组数据
    // rdCANfile << "      time: " << setprecision(16) << ros::Time::now().toSec() << endl; // us 需要 ros::Time::init();
    // rdCANfile << "     brake: " <<                    rd_CAN_info[0] << endl; 
    // rdCANfile << "     gears: " <<                    rd_CAN_info[1] << endl;
    // rdCANfile << "  st_angle: " << setprecision(6) << rd_CAN_info[2] << endl;
    // rdCANfile << "    st_DIR: " <<                    rd_CAN_info[3] << endl;
    // rdCANfile << "st_spd_DIR: " <<                    rd_CAN_info[4] << endl;
    // rdCANfile << "  st_speed: " << setprecision(6) << rd_CAN_info[5] << endl;
    // 将数据保存为txt 一行为一组数据(带时间戳)
    rdCANfile << setprecision(16) << ros::Time::now().toSec() << " "; // us 需要 ros::Time::init();
    rdCANfile <<                    rd_CAN_info[0] << " ";                 
    rdCANfile <<                    rd_CAN_info[1] << " "; 
    rdCANfile << setprecision(6) << rd_CAN_info[2] << " "; 
    rdCANfile <<                    rd_CAN_info[3] << " "; 
    rdCANfile <<                    rd_CAN_info[4] << " "; 
    rdCANfile << setprecision(6) << rd_CAN_info[5] << endl;   
    cb_mutex.unlock();   
}
void Car_info::ctrl_call_back(const vpm_msgs::ControlOrders::ConstPtr & ctrl_infos)
{
    
	// // CAN 数据的报头
// msg.ID;			// 定义要输入内容的类型(6c1方向盘信息, 6c3油门信息, 6c5刹车信息, 6c7档位信息)
// msg.SendType;	// 此设置以此为例无需变动
// msg.RemoteFlag;	// 此设置以此为例无需变动
// msg.ExternFlag;	// 此设置以此为例无需变动
// msg.DataLen;		// 定义Data数组的长度,每个Data是2个16进制位,也就是说方向盘信息DataLen=4
					// 油门信息DataLen=3, 刹车信息DataLen=3, 档位信息DataLen=3

/* msg.Data[0], msg.Data[1], msg.Data[2] // int mode, float angle, int speed
参数:
    int mode : 方向盘控制模式(十进制)
                2 是目标模式:直接指定方向盘位置 
                3 是左转模式:控制方向盘在当前位置左转多少度
                4 是右转模式:控制方向盘在当前位置右转多少度
    float angle : 方向盘转动角度(单位是度)
    int speed	: 方向盘转速(十进制)
                    0是慢速
                    1是中速
                    2是快速*/
    
    cb_mutex.lock(); 
    // 标志置1,在打印的函数里可以显示了
    ctrl_cb_flag = 1;
    // 将数据保存到局部缓冲区
int len = ctrl_infos->len;            // 接收数据的长度
vpm_msgs::ControlOrder temp[len];     // 定义缓冲区
    for (int i=0; i < len; ++i)       // 保存到缓冲区
	{ 
        temp[i] = ctrl_infos->control_orders[i]; 
    }
    // 将数据保存到全局缓冲区
    switch(temp[len-1].ID)
    {
        case 0x6c1: for(int i=0;i<temp[len-1].Data.size();++i) rd_CTRL_info[0][i] = temp[len-1].Data[i]; break;
        case 0x6c3: for(int i=0;i<temp[len-1].Data.size();++i) rd_CTRL_info[1][i] = temp[len-1].Data[i]; break;
        case 0x6c5: for(int i=0;i<temp[len-1].Data.size();++i) rd_CTRL_info[2][i] = temp[len-1].Data[i]; break;
        case 0x6c7: for(int i=0;i<temp[len-1].Data.size();++i) rd_CTRL_info[3][i] = temp[len-1].Data[i]; break;
        default: cout << "=====unknow ctrl ID.======" << endl; break;
    }
    cb_mutex.unlock();
    
    // 处理缓冲区数据, 存入txt
    for (int i=0; i< len; ++i) 
    {
        // 6c1方向盘信息
        if(temp[i].ID == 0x6c1) 
        {
            // ROS_INFO(" "); // 显示一下时间戳
            char mod_agl_sp[13] = {" "};
            sprintf(mod_agl_sp, " %d %4.2f %d", int(temp[i].Data[0]), temp[i].Data[1], int(temp[i].Data[2]));
            // cout << mod_agl_sp << endl;
            rdCTRLfile << setprecision(16) << ros::Time::now().toSec(); // 写入文件              
            rdCTRLfile << mod_agl_sp << endl;
        }
        // 油门信息
        else if(temp[i].ID == 0x6c3) ;
        // 刹车信息
        else if(temp[i].ID == 0x6c5) ;
        // 档位信息
        else if(temp[i].ID == 0x6c7) ;
    }
}
void Car_info::print_info()  
{
    static int cnt = 0;
    ++cnt;
    if(cnt==6)
    {
        cb_mutex.lock();
        cnt = 0;
        ROS_INFO(" "); // 显示一下时间戳
        // 只有调用了回调函数才会打印信息
        if(ctrl_cb_flag)
        {
            ctrl_cb_flag = 0;
            std::cout << "-------- CTRL输出信息 ---------"        << endl;
            std::cout << "   转向控制模式: " << rd_CTRL_info[0][0] << endl;
            std::cout << "   转向目标角度: " << rd_CTRL_info[0][1] << endl;
            std::cout << "   转向控制速度: " << rd_CTRL_info[0][2] << endl;
            std::cout << "-------------------------------"       << endl;
        }
        if(can_cb_flag)  
        {
            can_cb_flag = 0;          
            std::cout << "--------- CAN读回信息 ---------" << endl;
            std::cout << "           刹车: " << rd_CAN_info[0]  << endl;
            std::cout << "           档位: " << rd_CAN_info[1]  << endl;
            std::cout << "       转向角度: "  << rd_CAN_info[2] << endl;
            std::cout << "   转向转角方向: "  << rd_CAN_info[3]  << endl;
            std::cout << "   转向转速方向: "  << rd_CAN_info[4]  << endl;
            std::cout << "       转向转速: "  << rd_CAN_info[5] << endl;
            std::cout << "-------------------------------\n\n" << endl;
        }
        cb_mutex.unlock();
    }
}
void Car_info::show()
{
namespace plt = matplotlibcpp;
int start_time = ros::Time::now().toSec(); // ros起始时间
vector<queue<float>> vct_queue(5);         // time CAN CTRL 
float prev_ctrl = 1000;                         // 与控制的初始值抵消
float prev_can = 0;                             // 
const int temp_size = 200;                 // 画图缓冲大小
const std::map<std::string, std::string> keyword_arg1{
                                                        {"color", "C2"},
                                                        {"linewidth", "1.2"},
                                                        {"label","CAN"}};
const std::map<std::string, std::string> keyword_arg2{
                                                        {"color", "C1"},
                                                        {"linewidth", "1.2"},
                                                        {"label","CTRL"}};

    while(ros::ok())
    {
        // 读取时间戳
        vct_queue[0].push(ros::Time::now().toSec()-start_time);

        cb_mutex.lock();
        // 根据方向盘所处位置，CAN输出的转向角度，转换角度
        // 方向盘返回的是实际角度，但需要和rd_CAN_info[2]配合得出是左300还是右300度
        // rd_CAN_info[2]是方向盘当前所处位置是左(0)还是右(1)
        // 左为负数，右为正数
    int angle = 0;
        if(rd_CAN_info[3]) angle = rd_CAN_info[2];// 右
        else angle = -rd_CAN_info[2];             // 左
        // 保存CAN输出
        vct_queue[1].push(angle);
        // 保存控制节点的输出
        vct_queue[2].push(rd_CTRL_info[0][1]);
        // 一阶差分
        vct_queue[3].push(angle - prev_can);               // 保存CAN差分
        vct_queue[4].push(rd_CTRL_info[0][1] - prev_ctrl); // 保存CTRL差分
        prev_ctrl = rd_CTRL_info[0][1];
        prev_can = angle;
        cb_mutex.unlock();

        // 显示缓冲区大小的限制处理，没有超限继续存，超限pop之前的数据
    static int cnt = 1;
        if(cnt<temp_size) ++cnt; 
        else for(auto &v_q : vct_queue) v_q.pop(); // 当队列满之后开始删除
        // 拷贝缓冲区，依次存入显示的vector
    vector<queue<float>> temp_vct_queue(vct_queue);
    vector<vector<float>> v_data(temp_vct_queue.size());
        while(!temp_vct_queue[0].empty())
        {
            // 取出数据保存到最终显示的的vector  0-time 1-CAN 2-CTRL 3-diff1
            for(int i=0; i<v_data.size(); ++i)
            {
                // 如果是控制的输出需要剪掉1000的偏置
                if(i == 2) 
                    v_data[i].push_back(temp_vct_queue[i].front() - 1000);
                else 
                    v_data[i].push_back(temp_vct_queue[i].front());            
            }
            // 从队列中取出数据后删除
            for(auto &t_v_q : temp_vct_queue) t_v_q.pop();
        } 

        // 画animation
        plt::clf();
        plt::subplot(2, 1, 1);
        plt::suptitle("CAN and CTRL real-time steering angle data");
        plt::ylabel("L      angle(deg)      R");
        // plt::named_plot("CAN_read", v_data[0], v_data[1]); 
        // plt::named_plot("CTRL_out", v_data[0], v_data[2]);
        plt::plot(v_data[0], v_data[1], keyword_arg1); 
        plt::plot(v_data[0], v_data[2], keyword_arg2);
        // plt::ylim(-60, 60);
        plt::grid(1);
        plt::legend();

        plt::subplot(2, 1, 2);
        plt::ylabel("diff-1 (deg)");
        plt::xlabel("time(s)");
        // plt::named_plot("diff-1 of CAN_read", v_data[0], v_data[3]); 
        // plt::named_plot("diff-1 of CTRL_out", v_data[0], v_data[4]);
        plt::plot(v_data[0], v_data[3], keyword_arg1); 
        plt::plot(v_data[0], v_data[4], keyword_arg2);
        // plt::ylim(-40, 40);
        plt::grid(1);
        plt::legend();

        plt::pause(0.1); // 显示刷新20Hz 10Hz 5Hz
    }
}


// 保存车辆信息. 终端直接显示, 但只有当接收到can节点的信息时才保存数据到txt文件
// 车辆信息目前包括:
    // 0刹车状态:      1是踩下, 0是未踩下
    // 1档位状态:      0是P档, 1是R档, 2是N档, 3是D档, 6是B档
    // 2方向盘角度:     float 度
    // 3方向盘转角方向: 0是左, 1是右 (方向盘处于左边还是右边)
    // 4方向盘转速方向: 0是左, 1是右 (方向盘移动方向是左还是右)
    // 5方向盘转速:     度/秒
int main(int argc, char**argv)
{
    // 创建Car_info对象  // *******修改保存路径
    Car_info car_info(string(getenv("HOME"))+"/catkin_ws/src/read_steering/output_data/"); 
    // Car_info car_info(string(getenv("HOME"))+"/catkin_ws/src/vpm_control/read_steering/output_data/");  


    // 初始化ros节点
    ros::init(argc, argv, "read_steering_ros");
    ros::NodeHandle nh;
    ros::Time::init(); // 需要读取时间 
    // 初始化订阅器
	ros::Subscriber sub_CAN = 
			nh.subscribe("car_status", 1, &Car_info::can_call_back, &car_info); 
    ros::Subscriber sub_CTRL = 
        nh.subscribe("/control/control_orders", 1, &Car_info::ctrl_call_back, &car_info); 


    // message_filters::Subscriber<std_msgs::Float64MultiArray> sub_car(nh, "/car_status", 1000);
    // message_filters::Subscriber<std_msgs::Float64> sub_ctrl(nh, "/control/control_orders", 1000);
    // sub_car.registerCallback(&Car_info::can_call_back, &car_info);
    // sub_ctrl.registerCallback(&Car_info::ctrl_call_back, &car_info);

    // 接收频率
    ros::Rate loop_rate(15);
    // 画图，耗时间，如果只是保存txt则可有可无
    std::thread show_thread(&Car_info::show, &car_info);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        // 打印 终端显示car_info 如果只是保存txt则可有可无
        car_info.print_info(); 

    }
    show_thread.join();
    
    return 0;
}
