
#include "my_function.h"
using namespace DBoW3;
using namespace std;


// 读取path路径下的所有jpg文件名存入vector
void getFileNames(const string& path,vector<string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str())))
        return; // 如果路径打开，不存在
    while((ptr = readdir(pDir))!=0) {
        string image_name(ptr->d_name);
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)  
            // 只保留.jpg路径
            if (image_name.substr(image_name.size() - 4) == ".jpg")
                filenames.push_back(path + "/" + ptr->d_name);
    }
    closedir(pDir);
}

void loadTxt(const string& filename, vector<string>& names)
{
std::ifstream infile(filename.c_str());
    if (!infile){cerr<<"ERROR::: Failed to open the file <" <<filename << ">" << endl;exit(0);}
std::string one_row;
    while(infile >> one_row)
    {
        names.push_back(one_row);
    }
    infile.close(); // 关闭文件

    // for(int i = 0; i < names.size()/50; i++)  cout << "read txt content: " << names[i] << endl;
}

// 只提取一张图的feature
cv::Mat load_Feature(cv::Mat image, const int size_scale, const float split) throw (std::exception){
    //select detector
    cv::Ptr<cv::Feature2D> fdetector;
    fdetector=cv::ORB::create();

    vector<cv::KeyPoint> keypoints;
    cv::Mat feature;

    // ------------------预处理图片：resize split---------------------------
    // 速度排序： INTER_NEAREST 默认双线性插值，INTER_CUBIC
    image = image(cv::Range(0, int(image.rows*split)), cv::Range::all());
    resize(image, image,cv::Size(image.cols/size_scale, image.rows/size_scale), 0, 0);
    // -------------------------------------------------------------------

    fdetector->detectAndCompute(image, cv::Mat(), keypoints, feature);
    // cout<<"done detecting features" << endl;
    return feature;
}

// 功能：四舍五入（double），支持正负数
// dSrc ： 待四舍五入之数
// iBit ： 保留的小数位数。 0 - 不保留小数、1 - 保留一位小数
// 返回值：返回计算结果
double Round(double dSrc, int iBit)
{
	double retVal = 0.0;
	int  intTmp	= 0;
	// 若保留小数位数不正确
	if (0 > iBit)   return 0;
	//  若 为负数
	if (0 > dSrc)
	{
		// 首先转为正数
		dSrc *= -1;
		intTmp = (int)((dSrc + 0.5 / pow(10.0, iBit)) * pow(10.0, iBit));
		retVal = (double)intTmp / pow(10.0, iBit);
		// 再转为 负数
		retVal *= -1;
	}
	// 若为非负数
	else
	{
		intTmp = (int)((dSrc + 0.5 / pow(10.0, iBit)) * pow(10.0, iBit));
		retVal = (double)intTmp / pow(10.0, iBit);
	}
	// 返回计算结果
	return retVal;
}
