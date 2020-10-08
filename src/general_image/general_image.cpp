/**
 * ============================================================================
 *
 * Copyright (C) 2018, Hisilicon Technologies Co., Ltd. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */

#include "general_image.h"

#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include "hiaiengine/log.h"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include "tool_api.h"
#include "camera.h"

#include "hiaiengine/api.h"
#include "hiaiengine/data_type.h"
#include "hiaiengine/data_type_reg.h"
#include "data_type.h"


using hiai::Engine;
using namespace std;
using namespace ascend::atlas200dk;

namespace {
// output port (engine port begin with 0)
const uint32_t kSendDataPort = 0;

// sleep interval when queue full (unit:microseconds)
const __useconds_t kSleepInterval = 200000;

// get stat success
const int kStatSuccess = 0;
// image file path split character
const string kImagePathSeparator = ",";
// path separator
const string kPathSeparator = "/";

}

// register custom data type
HIAI_REGISTER_DATA_TYPE("ConsoleParams", ConsoleParams);
HIAI_REGISTER_DATA_TYPE("EngineTrans", EngineTrans);

HIAI_StatusT GeneralImage::Init(
    const hiai::AIConfig& config,
    const vector<hiai::AIModelDescription>& model_desc) {
  // do noting
  return HIAI_OK;
}

void GeneralImage::GetAllFiles(const string &path, vector<string> &file_vec) {
  // split file path
  vector<string> path_vector;
  SplitPath(path, path_vector);

  for (string every_path : path_vector) {
    // check path exist or not
    if (!IsPathExist(path)) {
      ERROR_LOG("Failed to deal path=%s. Reason: not exist or can not access.",
                every_path.c_str());
      continue;
    }

    // get files in path and sub-path
    GetPathFiles(every_path, file_vec);
  }

}

bool GeneralImage::IsDirectory(const string &path) {
  // get path stat
  struct stat buf;
  if (stat(path.c_str(), &buf) != kStatSuccess) {
    return false;
  }

  // check
  if (S_ISDIR(buf.st_mode)) {
    return true;
  } else {
    return false;
  }
}

bool GeneralImage::IsPathExist(const string &path) {
  ifstream file(path);
  if (!file) {
    return false;
  }
  return true;
}

void GeneralImage::SplitPath(const string &path, vector<string> &path_vec) {
  char *char_path = const_cast<char*>(path.c_str());
  const char *char_split = kImagePathSeparator.c_str();
  char *tmp_path = strtok(char_path, char_split);
  while (tmp_path) {
    path_vec.emplace_back(tmp_path);
    tmp_path = strtok(nullptr, char_split);

  }
}

void GeneralImage::GetPathFiles(const string &path, vector<string> &file_vec) {
  struct dirent *dirent_ptr = nullptr;
  DIR *dir = nullptr;
  if (IsDirectory(path)) {
    dir = opendir(path.c_str());
    while ((dirent_ptr = readdir(dir)) != nullptr) {
      // skip . and ..
      if (dirent_ptr->d_name[0] == '.') {
        continue;
      }

      // file path
      string full_path = path + kPathSeparator + dirent_ptr->d_name;
      // directory need recursion
      if (IsDirectory(full_path)) {
        GetPathFiles(full_path, file_vec);
      } else {
        // put file
        file_vec.emplace_back(full_path);
      }
    }
  } else {
    file_vec.emplace_back(path);
  }
}

bool GeneralImage::ArrangeImageInfo(shared_ptr<EngineTrans> &image_handle,
                                    const string &image_path) {
  // read image using OPENCV
  cv::Mat bgr_mat = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
  if (bgr_mat.empty()) {
    ERROR_LOG("Failed to deal file=%s. Reason: read image failed.",
              image_path.c_str());
    return false;
  }

  // set property
  image_handle->image_info.path = image_path;
  image_handle->image_info.width = bgr_mat.cols;
  image_handle->image_info.height = bgr_mat.rows;

  // set image data
  uint32_t size = bgr_mat.total() * bgr_mat.channels();
  u_int8_t *image_buf_ptr = new (nothrow) u_int8_t[size];
  if (image_buf_ptr == nullptr) {
    HIAI_ENGINE_LOG("new image buffer failed, size=%d!", size);
    ERROR_LOG("Failed to deal file=%s. Reason: new image buffer failed.",
              image_path.c_str());
    return false;
  }

  error_t mem_ret = memcpy_s(image_buf_ptr, size, bgr_mat.ptr<u_int8_t>(),
                             bgr_mat.total() * bgr_mat.channels());
  if (mem_ret != EOK) {
    delete[] image_buf_ptr;
    ERROR_LOG("Failed to deal file=%s. Reason: memcpy_s failed.",
              image_path.c_str());
    image_buf_ptr = nullptr;
    return false;
  }

  image_handle->image_info.size = size;
  image_handle->image_info.data.reset(image_buf_ptr,
                                      default_delete<u_int8_t[]>());
  return true;
}

bool GeneralImage::SendToEngine(const shared_ptr<EngineTrans> &image_handle) {
  // can not discard when queue full
  HIAI_StatusT hiai_ret;
  do {
    hiai_ret = SendData(kSendDataPort, "EngineTrans",
                        static_pointer_cast<void>(image_handle));
    // when queue full, sleep
    if (hiai_ret == HIAI_QUEUE_FULL) {
      HIAI_ENGINE_LOG("queue full, sleep 200ms");
      usleep(kSleepInterval);
    }
  } while (hiai_ret == HIAI_QUEUE_FULL);

  // send failed
  if (hiai_ret != HIAI_OK) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "call SendData failed, err_code=%d", hiai_ret);
    return false;
  }
  return true;
}


// ----------------------------------------------------------------------------
// 自定义将YUV转jpg保存的函数
// int SaveImage(const string& path, CameraData& camera_data)
cv::Mat SaveImage(const string& path, const shared_ptr<EngineTrans>& image_handle)
{
    int size = image_handle->image_info.size;
    cv::Mat yuvImg;
    cv::Mat bgrImg;
    yuvImg.create(image_handle->image_info.height * 3 / 2,
                    image_handle->image_info.width,
                    CV_8UC1);   
    memcpy(yuvImg.data, &(*(image_handle->image_info.data)), size * sizeof(unsigned char));
    
    cv::cvtColor(yuvImg, bgrImg, CV_YUV420sp2RGB);
    
    // bool save_ret = cv::imwrite(path, bgrImg);
    // if (!save_ret) {
    //     ERROR_LOG("Failed to deal file=%s. Reason: save camera image failed.",
    //                 path.c_str());
    //    return;
    // }  
    return bgrImg;
}
// ----------------------------------------------------------------------------



HIAI_IMPL_ENGINE_PROCESS("general_image",
    GeneralImage, INPUT_SIZE) {
  HIAI_StatusT ret = HIAI_OK;

  // Step1: check arg0
  if (arg0 == nullptr) {
    ERROR_LOG("Failed to deal file=nothing. Reason: arg0 is empty.");
    return HIAI_ERROR;
  }

  // Step2: get all files
  shared_ptr<ConsoleParams> console_param = static_pointer_cast<ConsoleParams>(
      arg0);


// 1. -------------------------- 初始化摄像头 -----------------------------
using namespace ascend::atlas200dk;
    // 初始化摄像头的参数，摄像头参数在这里手动配置
    CameraPara camera_para;
    camera_para.channel_id        = 0;
    camera_para.fps               = 20;
    // YUV420SP： widthStride * heightStride * 3 / 2
    camera_para.image_format      = CAMERA_IMAGE_YUV420_SP; // format of yuv CAMERA_IMAGE_YUV420_SP
    camera_para.resolution.width  = 704;   // 704 1920
    camera_para.resolution.height = 576;   // 576 1080
    camera_para.timeout           = 0;
    camera_para.capture_obj_flag  = 0;      // media type, IMAGE: image [0], VIDEO: video [1]  jpg or h264

    // 初始化摄像头
    Camera camera(camera_para);
    ret = camera.Init();
    if (ret != CameraInitOk) {
        camera.PrintErrorInfo(ret);
        return ret;
    } else {  // print to terminal
        cerr << "[INFO] Success to open camera["
            << camera.GetChannelId() << "],and start working."
            << endl;
        ASC_LOG_INFO("Success to open camera[%d],and start working.",
                    camera.GetChannelId());
    }

// 2. -------------------- 读取一帧图像数据 并处理后发送 ---------------------
// 将摄像头读取的图片保存最后一张到 path，image Engine 为 "./99.jpg"，
// 处理结果在post Engine 中为 "./999.jpg"
string path = "./99.jpg";
uint32_t yuv_size = camera.GetImgSize();
shared_ptr<u_int8_t> camera_data_ptr(new u_int8_t[yuv_size], default_delete<u_int8_t[]>());

// 2.1 ---------------- 读取一帧摄像头数据 YUV 格式 ------------------
while (camera.CaptureCamera(camera_data_ptr) == CameraRunOk) {
    cout << "\n---- frame_id: " << camera.GetFrameId() << endl;

// 2.2 -------------------- YUV 到 BGR 转换 -----------------------
    cv::Mat yuvImg, bgr_mat;
    yuvImg.create(camera._camera_para.resolution.height * 3 / 2,
                    camera._camera_para.resolution.width, CV_8UC1); 
    error_t mem_ret = memcpy_s(yuvImg.data, yuv_size, camera_data_ptr.get(), yuv_size);
    if (mem_ret != EOK) {
        ERROR_LOG("Failed to 'memcpy_s' camera_data_ptr to yuvImg.");
        return false;
    }    
    cv::cvtColor(yuvImg, bgr_mat, CV_YUV420sp2RGB);
    if (bgr_mat.empty()) {
        cout << "----- bgr_mat empty.-----" << endl;
        continue;
    }    
    // 写入到当前路径，因为 post Engine 需要在原图上画框框
    bool save_ret = cv::imwrite(path, bgr_mat);
    if (!save_ret) {
        ERROR_LOG("Failed to deal file=%s. Reason: save image failed.",
                path.c_str());
        return HIAI_ERROR;
    }

// 2.3 ---------------- 将 BGR 数据拷贝到发送缓冲区 -------------------
    // 申请发送缓冲区 buff
    uint32_t bgr_size = bgr_mat.total() * bgr_mat.channels(); // 1216512
    shared_ptr<u_int8_t> send_buff(new u_int8_t[bgr_size], default_delete<u_int8_t[]>());
    // 将转换好的 bgr 格式数据 cpy 到缓冲区
    mem_ret = memcpy_s(send_buff.get(), bgr_size, bgr_mat.ptr<u_int8_t>(), 
                                bgr_mat.total() * bgr_mat.channels());

// 2.4 -------------------- 设置 EngineTrans -----------------------
    // 申请 EngineTrans
    shared_ptr<EngineTrans> image_handle = nullptr;
    MAKE_SHARED_NO_THROW(image_handle, EngineTrans);
    if (image_handle == nullptr) {
        ERROR_LOG("Failed to deal file=%s. Reason: new EngineTrans failed.",
                    path.c_str());
        continue;
    }

    // 初始化 EngineTrans
    image_handle->image_info.size = bgr_size;
    image_handle->image_info.data = send_buff;
    image_handle->image_info.path = path;
    image_handle->image_info.height = bgr_mat.rows;
    image_handle->image_info.width = bgr_mat.cols;

    // send data to inference engine
    image_handle->console_params.input_path = path;
    image_handle->console_params.model_height = console_param->model_height;
    image_handle->console_params.model_width = console_param->model_width;
    image_handle->console_params.output_path = console_param->output_path;    

// 3. ----------------------- 发送数据图像 --------------------------------
    if (!SendToEngine(image_handle)) {
        ERROR_LOG("general_image send data failed.");
        continue;
    }

    // sleep
    usleep(kSleepInterval);
}

// 4. -------------------- 结束发送时，就发送结束标志出去 ---------------------
    // Step4: send finished data
    shared_ptr<EngineTrans> image_handle = nullptr;
    MAKE_SHARED_NO_THROW(image_handle, EngineTrans);
    if (image_handle == nullptr) {
        ERROR_LOG("Failed to send finish data. Reason: new EngineTrans failed.");
        ERROR_LOG("Please stop this process manually.");
        return HIAI_ERROR;
    }
    image_handle->is_finished = true;
    if (SendToEngine(image_handle)) {
        return HIAI_OK;
    }
    ERROR_LOG("Failed to send finish data. Reason: SendData failed.");
    ERROR_LOG("Please stop this process manually.");
    return HIAI_ERROR;
}
