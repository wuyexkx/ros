#include <iostream>
#include <vector>
// DBoW3
#include "DBoW3.h"
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#ifdef USE_CONTRIB
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#endif
#include "DescManip.h"
#include <sys/types.h>
#include <dirent.h>
#include <opencv2/imgproc/imgproc.hpp>


using namespace DBoW3;
using namespace std;

// 从指定路经读取.jpg文件的路经 到filenames
void getFileNames(string path, vector<string>& filenames)
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

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// extended surf gives 128-dimensional vectors
const bool EXTENDED_SURF = false;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void wait()
{
    cout << endl << "Press enter to continue" << endl;
    getchar();
}

// 根据路经返回所有图片的特征
vector< cv::Mat >  loadFeatures( std::vector<string> path_to_images,string descriptor="", 
                                 int size_scale = 1, float split = 0.6) throw (std::exception){
    //select detector
    cv::Ptr<cv::Feature2D> fdetector;
    if (descriptor=="orb")        fdetector=cv::ORB::create();
    else if (descriptor=="brisk") fdetector=cv::BRISK::create();
#ifdef OPENCV_VERSION_3
    else if (descriptor=="akaze") fdetector=cv::AKAZE::create();
#endif
#ifdef USE_CONTRIB
    else if(descriptor=="surf" )  fdetector=cv::xfeatures2d::SURF::create(400, 4, 2, EXTENDED_SURF);
#endif

    else throw std::runtime_error("Invalid descriptor");
    assert(!descriptor.empty());
    vector<cv::Mat> features;

    cout << "Extracting   features..." << endl;
    for(size_t i = 0; i < path_to_images.size(); ++i)
    {
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        cout << "reading image: " << path_to_images[i] << endl;
        cv::Mat image = cv::imread(path_to_images[i]);
        if(image.empty())throw std::runtime_error("Could not open image"+path_to_images[i]);
        cv::Mat grayImage; // 转灰度
        cv::cvtColor(image, grayImage, CV_BGR2GRAY);

        // --------------预处理图片：resize split-------------------
        // 速度排序： INTER_NEAREST 默认双线性插值，INTER_CUBIC
        grayImage = grayImage(cv::Range(0, int(grayImage.rows*split)), cv::Range::all());
        resize(grayImage, grayImage,cv::Size(grayImage.cols/size_scale, grayImage.rows/size_scale), 0, 0);
        // -------------------------------------------------------

        cout<<"extracting features"<<endl;
        fdetector->detectAndCompute(grayImage, cv::Mat(), keypoints, descriptors);
        features.push_back(descriptors);
        cout<<"done detecting features"<<endl;
    }
    return features;
}

// ----------------------------------------------------------------------------
// 传入图像的特征集，配置聚类树的分支树(k)，以及深度(l)，
// 调用create进行聚类即得到Vocabulary。 
// 可以将得到的Vocabulary保存成文件，以便后面使用。
void saveVocCreation(const vector<cv::Mat> &features, string save_path)
{
    // branching factor and depth levels
    const int k = 9;
    const int L = 3;
    const WeightingType weight = TF_IDF;
    const ScoringType score = L1_NORM;

    // 实例化Vocabulary对象voc
    DBoW3::Vocabulary voc(k, L, weight, score);

    cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
    // 根据特征创建词典（词汇集合）
    voc.create(features);
    cout << "create voc done." << endl;
    cout << "Vocabulary information: " << endl << voc << endl << endl;

    // save the vocabulary to disk
    cout << endl << "Saving vocabulary..." << endl;

    voc.save(save_path);
    cout << "=====save voc Done, in " << "'" << save_path << "'=====" << endl;
}

// ----------------------------------------------------------------------------
// 有了Vocabulary后，就可以构建一个Database方便图像的查找。
// 需要前面的Vocabulary和特征集来创建Database，创建完后，也可以将其保存为本地文件。
// 有了Database后，可以其调用query方法，来查找数据库中是否有相类似的图片。
void saveDatabase(const  vector<cv::Mat > &features, string voc_path, string db_path)
{
    cout << "Creating a mould database..." << endl;

    // load the vocabulary from disk
    Vocabulary voc(voc_path);

    Database db(voc, false, 0); // false = do not use direct index
    // (so ignore the last param)
    // The direct index is useful if we want to retrieve the features that
    // belong to some vocabulary node.
    // db creates a copy of the vocabulary, we may get rid of "voc" now

    // add images to the database
    for(size_t i = 0; i < features.size(); i++)
        db.add(features[i]);

    cout << "add features to db done!" << endl;
    cout << "Database information: " << endl << db << endl;

    cout << "Saving database..." << endl;
    db.save(db_path);
    cout << "=====save db Done, in " << "'" << db_path << "'=====" << endl;

}


// 功能：根据图片创建用于图像检索的数据库
// 会创建三个文件
// mould_voc.yml.gz
// mould_db.yml.gz 
// mould_names.txt
// 然后用一张图片来验证创建的数据库
int main()
{
    vector<string> goalimg_path(1);
    // 设置数据库保存的路径
    string database_path = "/home/wuyexkx/Desktop/dbow3_ros_imgRename/building_retrieve/database";
    string voc_path = database_path + "/mould_voc.yml.gz";
    string db_path = database_path + "/mould_db.yml.gz";
    string txt_path = database_path + "/mould_names.txt";
    // descriptors:brisk,surf,orb ,akaze(only if using opencv 3)
    string descriptor="orb";
    // mould_path是根据模板图片所在文件夹重命名（python完成）后的图片路径
    string mould_path = database_path + "/images";
    // 选取一张图片测试创建的数据库
    goalimg_path[0] = database_path + "/4.jpg";


    // 读入所有.jpg文件的路径 到img_names中
    vector<string> img_names;
    getFileNames(mould_path, img_names);
    // unsigned int img_num = img_names.size();
    // for(int i = 0; i < img_name.size(); i++) cout<<img_name[i]<<endl;
    try{
        // ------------------根据mould创建数据库-----------------------
        // 提取图库特征
        vector< cv::Mat > features = loadFeatures(img_names,descriptor);
        // 创建视觉词典
        saveVocCreation(features, voc_path);
        // 保存数据库
        saveDatabase(features, voc_path, db_path);

        // 保存ima_names为 mould_names.txt
         ofstream outfile(txt_path, ios::out);
         unsigned int row_num = img_names.size();
         for(unsigned i=0; i < row_num; i++)
         {// 将img_names按行写入txt 
            outfile << img_names[i] << endl;             
         }
         outfile.close(); 
        // ---------------------------------------------------------


        // -------------------验证数据库正确与否-----------------------
        // -----读取txt的每一行 存入names
        vector<string> names;
        // 打开txt文件
        ifstream infile(txt_path);
        if (!infile){cerr<<"ERROR::: Failed to open the file <" <<txt_path << ">" << endl;exit(0);}
        string one_img_name;
        // 将打开的文件infile按行写入names
        while(infile >> one_img_name)
        {   
            names.push_back(one_img_name);
        }
        infile.close(); 
        for(unsigned i = 0; i < names.size(); i++)  cout << "read path from txt: " << names[i] << endl;

        // -----选取一张图片,验证检索
        vector< cv::Mat > features1 = loadFeatures(goalimg_path, descriptor);
        // 检索
        QueryResults ret;
        cout << "one-img Retrieving database ..." << endl;
        Database db2(db_path);
        cout << "load db done! This is: " << endl << db2 << endl;
        // 单个特征在db中查询，结果保存到ret，10表示返回Score最好的10个结果
        int rst_nums = 10;
        db2.query(features1[0], ret, rst_nums);
        cout << "Searching for Image top best " << ret << endl;
        cout << endl;
        cout << "Corresponding imgs:" << endl;
        for(int i=0;i<rst_nums; i++) { cout << i << ": " << names[ret[i].Id] << endl; }
        // --------------------------------------------------------

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }

    return 0;
}

