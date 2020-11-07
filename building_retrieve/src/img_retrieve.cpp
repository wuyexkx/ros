#include <iostream>
#include <vector>
#include <string>
#include "sys/time.h"
#include "my_function.h"

using namespace DBoW3;
using namespace std;


// 功能：输入一张图片，检索与数据库里的相似度
// 数据库来自于save_database.cpp保存的mould_db.yml.gz和mould_names.txt
// 输出检索Score结果  对应图片路经  检索速度  帧率
int main()
{   
    vector<string> names;
    string database_path = "/home/wuyexkx/Desktop/building_retrieve/database";
    // db数据库路经
    Database db(database_path + "/mould_db.yml.gz");
    // 读取txt，内容存放到names  
    loadTxt(database_path + "/mould_names.txt", names); 
    // 一张图片的路径
    string goalimg_path = database_path + "/88.jpg";  
    QueryResults ret;
    cv::Mat grayImage;
    cv::Mat feature;
    cv::Mat image = cv::imread(goalimg_path);

    try{
        
        struct timeval startTime, extractTime, endTime;
        gettimeofday(&startTime, NULL);
        // image 传入mat图片到image
        if(image.empty())throw std::runtime_error("Read the image is empty!");
        // 转灰度
        cv::cvtColor(image, grayImage, CV_BGR2GRAY);
        feature = load_Feature(grayImage);
        gettimeofday(&extractTime, NULL);

        // 单张图特征在db中查询，结果保存到ret
        int rst_nums = 10;
        db.query(feature, ret, rst_nums); 
        gettimeofday(&endTime, NULL);

        // 输出前10检索结果
        cout << endl;
        for(int i=0;i<rst_nums; i++) 
        { 
            cout << i << ": " << ret[i].Score << "--" << names[ret[i].Id] << endl;
        }
        
        // 判断条件，当Score满足条件是才输出结果
        string result;
        if(ret[0].Score > 0.3)
        {
            string maxvlupath = names[ret[0].Id];
            result = maxvlupath.substr(maxvlupath.rfind("/")+1, 
                            (maxvlupath.rfind("-")-(maxvlupath.rfind("/")+1)));
        }    
        else  result = "None";
        cout << "\nOutput: " << result << endl;

        cout << endl;
        // 计算检索一张图所需时间及帧率
        // 时间： 总时间=特征提取时间+检索时间
        // 帧率： 1秒可以有多少张图片 检索当前数据库
        double a = (double)(endTime.tv_usec-startTime.tv_usec)/1000000;
        double b = (double)(extractTime.tv_usec-startTime.tv_usec)/1000000;
        double c = (double)(endTime.tv_usec-extractTime.tv_usec)/1000000;
        cout << "totalTime   : " << Round(a, 4) << "s--FPS(1000): " << Round(1/a, 1)  << endl;
        cout << "extractTime : " << Round(b, 4) << "s--FPS(1000): " << Round(1/b, 1)  << endl;
        cout << "retrieveTime: " << Round(c, 4) << "s--FPS(1000): " << Round(1/c, 1)  << endl;
                 
    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
    return 0;
}
