
#ifndef __MY_FUNCTION__
#define __MY_FUNCTION__

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
#include <fstream>
#include<time.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

// extended surf gives 128-dimensional vectors
const bool EXTENDED_SURF = false;


void getFileNames(const string& path,vector<string>& filenames);
void loadTxt(const string& filename, vector<string>& names);
// size_scale(1 为原图, 4为缩小4倍)  split(1为不切分, 0.4为保留40%上部分)
cv::Mat load_Feature(cv::Mat image, const int size_scale = 1, const float split = 0.6) throw (std::exception);
double Round(double, int);


#endif
