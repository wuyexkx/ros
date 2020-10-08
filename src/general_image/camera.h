#ifndef ASCEND_ATLAS200DK_CAMERA_H_
#define ASCEND_ATLAS200DK_CAMERA_H_

#include <string>
#include <memory>
#include "hiaiengine/log.h"

extern "C" {
#include "driver/peripheral_api.h"
}


namespace ascend {
namespace atlas200dk {

// used for record info-level log info to dlog
#ifndef ASC_LOG_INFO
#define ASC_LOG_INFO(fmt, ...) \
  HIAI_ENGINE_LOG( "[%s:%d] " fmt "\n", __FILE__, __LINE__, \
            ##__VA_ARGS__)
#endif

// used for record warning-level log info to dlog
#ifndef ASC_LOG_WARN
#define ASC_LOG_WARN(fmt, ...) \
  HIAI_ENGINE_LOG(HIAI_GRAPH_WARNING_CODE, \
     "[%s:%d] " fmt "\n", __FILE__,  __LINE__, ##__VA_ARGS__)
#endif

// used for record error-level log info to dlog
#ifndef ASC_LOG_ERROR
#define ASC_LOG_ERROR(fmt, ...) \
  HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, \
     "[%s:%d] " fmt "\n", __FILE__,  __LINE__, ##__VA_ARGS__)
#endif


// parameter maximum length
const int kMaxParamLength = 4096;
// used for image size conversion
const int kYuv420spSizeNumerator = 3;
// used for image size conversion
const int kYuv420spSizeDenominator = 2;

const int CameraReturnInvalid = 0;
const int CameraReturnValid = 1;

enum CameraErrorID {
  CameraInitOk = 0,
  CameraRunOk = 0,
  CameraInitError = -1,
  CameraStatusError = -2,
  CameraOpenError = -3,
  CameraSetFpsError = -4,
  CameraSetFormatError = -5,
  CameraSetResolutionError = -6,
  CameraSetWorkModeError = -7,
  CameraGetInfoError = -8,
  CameraMediaStatusError = -9,
};

// used for error info description
struct ErrorDescription {
  int code;
  std::string code_info;
};


typedef struct {
    unsigned int channel_id;
    int fps;
    // format of yuv
    CameraImageFormat image_format;
    // width height
    CameraResolution resolution;
    int size;
    int timeout;
    // jpg or h264
    int capture_obj_flag;    
} CameraPara;


class Camera {
public:

    Camera(CameraPara& para);

    // init driver of camera and open camera.
    int Init();
    
    int CaptureCamera(std::shared_ptr<u_int8_t>& camera_data_ptr);
    int GetChannelId() const;
    int GetUserTimeout() const;
    int GetImgSize() const;
    unsigned int GetFrameId() const;
    void PrintErrorInfo(int code) const;

    CameraPara _camera_para;
private:

    // frame id.
    unsigned int _frame_id; 
    int _channel_id;
    // size of one frame data from camera.
    int _image_size;

    // CameraPara _camera_para;
};


}
}

#endif
