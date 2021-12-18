#ifndef _DH_CAMERA_H_
#define _DH_CAMERA_H_

#include <string>
#include "GxIAPI.h"
#include "DxImageProc.h"

class DHCamera {
public:
    enum ErrorCode {
        SUCCESS = 0,
        ERR_DEVICE_NOT_FOUND = -1,
        ERR_CAMERA_INTERNAL_ERROR = 1,
        ERR_UNSUPPORTED_CAMERA = -2
    };

    DHCamera();

    DHCamera(const DHCamera &) = delete;

    DHCamera(const DHCamera &&) = delete;

    ~DHCamera();

    ErrorCode OpenCamera(uint32_t device_id = 1);

    ErrorCode StartAcquisition();

    ErrorCode StopAcquisition();

    ErrorCode CloseCamera();

    std::string GetVendorName();

    std::string GetModelName();

    std::string GetSerialNumber();

    std::string GetDeviceVersion();

    static std::string GetErrorInfo(GX_STATUS);

    inline static unsigned int GetCameraNumber() {
        return camera_number_;
    }

private:
    static void *ThreadProc(void *);

    static unsigned int camera_number_;

    bool PixelFormatConvert(PGX_FRAME_BUFFER);

    GX_DEV_HANDLE device_ = nullptr;
    int64_t color_filter_ = GX_COLOR_FILTER_NONE;
    int64_t payload_size_ = 0;
    unsigned char *raw_8_to_rgb_24_cache_ = nullptr;
    unsigned char *raw_16_to_8_cache_ = nullptr;
    bool thread_alive_ = false;
    pthread_t thread_id_ = 0;
    PGX_FRAME_BUFFER frame_buffer_ = nullptr;
};

#endif  // _DH_CAMERA_H_
