#include "camera.h"

// #include <stdio.h>
// #include <stdlib.h>
#include "hiaiengine/log.h"
#include "stdio.h"
#include "stdlib.h"
#include <sys/types.h>    
#include <sys/stat.h>    
#include <fcntl.h>
#include "pthread.h"
#include <unistd.h>
#include "peripheral_api.h"

using namespace std;
namespace ascend {
namespace atlas200dk {

// 用 CameraPara 去初始化 Camera 内部的参数 _camera_para
Camera::Camera(CameraPara& para)
{ 
    // Initialization instance
    _camera_para = para;
    _image_size = _camera_para.resolution.width     \
                * _camera_para.resolution.height    \
                * kYuv420spSizeNumerator            \
                / kYuv420spSizeDenominator;
    para.size = _image_size;
    _frame_id = 0;   
    _channel_id = para.channel_id;
}

// init driver of camera and open camera.
int Camera::Init()
{
    int ret = CameraInitOk;  

    // init driver of camera
    ret = MediaLibInit();
    if (ret == LIBMEDIA_STATUS_FAILED) {
        return CameraMediaStatusError;
    }

    // check camera status
    CameraStatus status = QueryCameraStatus(_camera_para.channel_id);
    if (status != CAMERA_STATUS_CLOSED) {
    ASC_LOG_ERROR("Camera[%d] is unavailable,status is %d.",
                    _camera_para, status);
        return CameraStatusError;
    }  

    // open camera
    ret = OpenCamera(_camera_para.channel_id);
    if (ret == CameraReturnInvalid) {
    ASC_LOG_ERROR("Camera[%d] open failed.", _camera_para.channel_id);
        return CameraOpenError;
    }

    // set fps
    ret = SetCameraProperty(_camera_para.channel_id, CAMERA_PROP_FPS,
                            &(_camera_para.fps));
    if (ret == CameraReturnInvalid) {
    ASC_LOG_ERROR("Camera[%d] set fps[%d] failed.",
                    _camera_para.channel_id, _camera_para.fps);
        return CameraSetFpsError;
    }

    // set image format
    ret = SetCameraProperty(_camera_para.channel_id,
                            CAMERA_PROP_IMAGE_FORMAT,
                            &(_camera_para.image_format));
    if (ret == CameraReturnInvalid) {
    ASC_LOG_ERROR("Camera[%d] set image_fromat[%d] failed.",
                    _camera_para.channel_id,
                    _camera_para.image_format);
        return CameraSetFormatError;
    }

    // set image resolution .
    ret = SetCameraProperty(_camera_para.channel_id,
                            CAMERA_PROP_RESOLUTION,
                            &(_camera_para.resolution));
    if (ret == CameraReturnInvalid) {
    ASC_LOG_ERROR("Camera[%d] set resolution[%d x %d] failed.",
                    _camera_para.channel_id,
                    _camera_para.resolution.width,
                    _camera_para.resolution.height);
        return CameraSetResolutionError;
    }

    CameraCapMode camera_mode = CAMERA_CAP_ACTIVE;
    // set work mode
    ret = SetCameraProperty(_camera_para.channel_id,
                            CAMERA_PROP_CAP_MODE, &(camera_mode));
    if (ret == CameraReturnInvalid) {
    ASC_LOG_ERROR("Camera[%d] set cap mode[%d] failed.",
                    _camera_para.channel_id, camera_mode);
        return CameraSetWorkModeError;
    }

    return CameraInitOk;
}

int Camera::CaptureCamera(shared_ptr<u_int8_t>& camera_data_ptr)
{
    int ret = CameraReturnValid;
    int result = CameraRunOk;
    int size = _image_size;

    // read info from camera
    ret = ReadFrameFromCamera(_camera_para.channel_id,
                            (void *)(camera_data_ptr.get()), &size);

    if ((ret == CameraReturnValid) && (size == _image_size)) {
        // success to get a frame from camera
        _frame_id++;
        result = CameraRunOk;
    } else {  // failed to read a frame data from camera
        CameraStatus status = QueryCameraStatus(_camera_para.channel_id);
        ASC_LOG_ERROR("Camera[%d] get image failed status is %d.",
                     _camera_para.channel_id, status);
        result = CameraGetInfoError;
    }
    return result;    
}

int Camera::GetChannelId() const {
  return _camera_para.channel_id;
}

int Camera::GetUserTimeout() const {
  return _camera_para.timeout;
}
int Camera::GetImgSize() const {
    return _image_size;
}
unsigned int Camera::GetFrameId() const {
    return _frame_id;
}

void Camera::PrintErrorInfo(int code) const {

    static ErrorDescription camera_description[] = { 
        { CameraInitError,          "Failed to initialize camera." }, 
        { CameraStatusError,        "The camera status is not correct ." }, 
        { CameraOpenError,          "Failed to open camera." }, 
        { CameraSetFpsError,        "Failed to set fps." }, 
        { CameraSetFormatError,     "Failed to set format." }, 
        { CameraSetResolutionError, "Failed to set resolution." }, 
        { CameraSetWorkModeError,   "Failed to set work mode." }, 
        { CameraGetInfoError,       "Failed to get info from camera." }, 
    };

  // find same errorcode and get error description
  int num = sizeof(camera_description) / sizeof(ErrorDescription);
    for (int i = 0; i < num; i++) {
        if (code == camera_description[i].code) {
            std::cerr << "[ERROR] " << camera_description[i].code_info.c_str() << std::endl;
            return;
        }
    }

  std::cerr << "[ERROR] Other error." << std::endl;
}


}
}


