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
#include <vector>
#include <pwd.h>
#include "vpm_msgs/ControlOrder.h"
#include "vpm_msgs/ControlOrders.h"
#include "matplotlibcpp.h"


using namespace std;
using namespace message_filters;


std::string roundNum(double r, int precision)
{
    std::stringstream buff;
    buff << setprecision(precision) << r;
    return buff.str();
}

class Car_info
{
public:
    Car_info(const std::string& txt_path)
    : rd_CAN_info(20, -1), rd_CTRL_info(20, 1000), cb_flag(0)
    { 
        time_t now = time(0);
        tm *ltm = localtime(&now);
        // 输出 tm 结构的各个组成部分
        string data = to_string(1900 + ltm->tm_year) + "_" + 
                      to_string(1 + ltm->tm_mon) + "_" +
                      to_string(ltm->tm_mday) + "_" + 
                      to_string(ltm->tm_hour) + "_" + 
                      to_string(ltm->tm_min) + "_" + 
                      to_string(ltm->tm_sec);
        rdCANfile.open(txt_path + data + "_CAN_info.txt", ios::trunc);//打开文件，清空
        rdCTRLfile.open(txt_path + data + "_CTRL_info.txt", ios::trunc);//打开文，清空

        ROS_INFO(("\n=========== save txt path: " + txt_path).c_str());
    }
    ~Car_info() { rdCANfile.close(); rdCTRLfile.close(); }
    inline void call_back(const std_msgs::Float64MultiArray::ConstPtr& car_info_msg);
    inline void ctrl_call_back(const vpm_msgs::ControlOrders::ConstPtr & ctrl_infos);
    inline void print_info();
    inline void show(int temp_size = 250);

private:
    std::ofstream rdCANfile;
    std::ofstream rdCTRLfile;
    std::vector<double> rd_CAN_info;
    std::vector<double> rd_CTRL_info;
    std::mutex cb_mutex;
    int cb_flag;
};
void Car_info::call_back(const std_msgs::Float64MultiArray::ConstPtr& car_info_msg)
{
unsigned len = car_info_msg->data.size();	// 取得消息数组的长度 (6)
    // 0刹车状态:      1是踩下, 0是未踩下
    // 1档位状态:      0是P档, 1是R档, 2是N档, 3是D档, 6是B档
    // 2方向盘角度:     float 度
    // 3方向盘转角方向: 0是左, 1是右 (方向盘处于左边还是右边)
    // 4方向盘转速方向: 0是左, 1是右 (方向盘移动方向是左还是右)
    // 5方向盘转速:     度/秒

    // 暂存接收缓冲数据
    cb_mutex.lock(); cb_flag = 1;
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
    
    // 将数据保存到缓冲区
    int len = ctrl_infos->len;        // 接收数据的长度
	vpm_msgs::ControlOrder temp[len];  // 定义缓冲区
    for (int i=0; i < len; ++i)       // 保存到缓冲区
	{ temp[i] = ctrl_infos->control_orders[i]; rd_CTRL_info[i] = temp[i].Data[1]; }

    // 处理缓冲区数据
    for (int i=0; i< len; ++i) 
    {
        // 6c1方向盘信息
        if(temp[i].ID == 0x6c1) 
        {
            // temp

            // ROS_INFO(" "); // 显示一下时间戳
            char mod_agl_sp[13] = {" "};
            sprintf(mod_agl_sp, " %d %4.2f %d", int(temp[i].Data[0]), temp[i].Data[1], int(temp[i].Data[2]));
            // cout << mod_agl_sp << endl;
            rdCTRLfile << setprecision(16) << ros::Time::now().toSec(); // 写入文件              
            rdCTRLfile << mod_agl_sp << endl;
            cout << "CTRL输出:" << mod_agl_sp << " (mode angle sp)" << endl;
        }
        // 油门信息
        else if(temp[i].ID == 0x6c3) ;
        // 刹车信息
        else if(temp[i].ID == 0x6c5) ;
        // 档位信息
        else if(temp[i].ID == 0x6c7) ;
    }
}
void Car_info::print_info()  // 回调函数33Hz, freq_div默认为4 所以显示为8Hz
{
static unsigned char cnt = 0;
    cb_mutex.lock();
    if(cb_flag)  
    {
        cb_flag = 0;          
        ROS_INFO(" "); // 显示一下时间戳
        std::cout << "--------- CAN读回信息 ---------" << endl;
        std::cout << "          刹车: "      << rd_CAN_info[0] << endl;
        std::cout << "          档位: "      << rd_CAN_info[1] << endl;
        std::cout << "    方向盘角度: "      << rd_CAN_info[2] << endl;
        std::cout << "方向盘转角方向: "      << rd_CAN_info[3] << endl;
        std::cout << "方向盘转速方向: "      << rd_CAN_info[4] << endl;
        std::cout << "    方向盘转速: "      << rd_CAN_info[5] << endl;
        std::cout << "--------------------------------\n\n" << endl;
    }
    cb_mutex.unlock();
}
void Car_info::show(int temp_size)
{
    namespace plt = matplotlibcpp;
    static int cnt = 1;
    static int start_time = ros::Time::now().toSec();

    static queue<float> q_CAN;
    static queue<float> q_CTRL;
    static queue<float> q_t;

    vector<float> v_CAN;
    vector<float> v_CTRL;
    vector<float> v_t;

    cb_mutex.lock();
    // 根据方向盘所处位置 转换角度
    int angle = 0;
    if(rd_CAN_info[3]) // 右转
        angle = rd_CAN_info[2]; 
    else               // 左转
        angle = -rd_CAN_info[2];
    q_CAN.push(angle);
    // 读取控制节点的输出
    q_CTRL.push(rd_CTRL_info[0]);
    // 读取时间戳
    q_t.push(ros::Time::now().toSec()-start_time);
    if(cnt<temp_size)  // 缓冲区计数
    { 
        ++cnt; 
    } 
    else
    {
        q_CAN.pop();      // 当队列满之后开始删除
        q_CTRL.pop(); 
        q_t.pop();
    }
    // 取出缓冲区
    queue<float> temp_q1(q_CAN);
    queue<float> temp_q2(q_CTRL);
    queue<float> temp_qt(q_t);
    while(!temp_q1.empty())
    {
        v_CAN.push_back(temp_q1.front());
        v_CTRL.push_back(temp_q2.front()-1000);
        v_t.push_back(temp_qt.front());
        temp_q1.pop();
        temp_q2.pop();
        temp_qt.pop();
    } 

    plt::clf();
    plt::ylim(-600, 600);
    plt::named_plot("CAN_read", v_t, v_CAN);
    plt::named_plot("CTRL_out", v_t, v_CTRL);
    plt::ylabel("L       angle(deg)       R");
    plt::xlabel("time(s)");
    plt::title("CTRL and CAN real-time steering angle");
    plt::grid(1);
    plt::legend();
    plt::pause(0.06); // 显示刷新20Hz 15Hz

    cb_mutex.unlock();
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
			nh.subscribe("car_status", 10, &Car_info::call_back, &car_info); 
    ros::Subscriber sub_CTRL = 
        nh.subscribe("/control/control_orders", 10, &Car_info::ctrl_call_back, &car_info); 


    // message_filters::Subscriber<std_msgs::Float64MultiArray> sub_car(nh, "/car_status", 1000);
    // message_filters::Subscriber<std_msgs::Float64> sub_ctrl(nh, "/control/control_orders", 1000);
    // sub_car.registerCallback(&Car_info::call_back, &car_info);
    // sub_ctrl.registerCallback(&Car_info::ctrl_call_back, &car_info);

    // 接收频率
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        // 终端显示car_info
        car_info.print_info(); 
        // random_shuffle(vct.begin(), vct.end());
        car_info.show();
    }

    // 关闭ros后需要保存文件
    car_info.~Car_info();
    
    return 0;
}
