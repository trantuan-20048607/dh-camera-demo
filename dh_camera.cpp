#include "dh_camera.h"
#include "GxIAPI.h"
#include "DxImageProc.h"
#include <string>
#include <iostream>
#include <stdexcept>

#define ACQ_BUFFER_NUM          5               ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64              ///< Qty. of data transfer block

// 检查各个步骤的返回值，开启数据流后使用
#define GX_CHECK_STATUS_WITH_STREAM(status_code)          \
if ((status_code) != GX_STATUS_SUCCESS) {                 \
    if (raw_16_to_8_cache_ != nullptr) {                  \
        delete[] raw_16_to_8_cache_;                      \
        raw_16_to_8_cache_ = nullptr;                     \
    }                                                     \
    if (rgb_8_to_rgb_24_cache_ != nullptr) {              \
        delete[] rgb_8_to_rgb_24_cache_;                  \
        rgb_8_to_rgb_24_cache_ = nullptr;                 \
    }                                                     \
    std::cout << GetErrorInfo(status_code) << std::endl;  \
    return ERR_CAMERA_INTERNAL_ERROR;                     \
}

// 全局相机计数，此数据操作与 device_ 是否为空紧密相关
unsigned int DHCamera::camera_number_ = 0;

DHCamera::DHCamera() {
    // 加载 .so 库
    if (GXInitLib() != GX_STATUS_SUCCESS) {
        throw std::runtime_error("GX libraries not found.");
    }
}

DHCamera::~DHCamera() {
    if (thread_alive_) {
        StopAcquisition();
    }
    if (device_ != nullptr) {
        CloseCamera();
    }
    GXCloseLib();
}

DHCamera::ErrorCode DHCamera::OpenCamera(uint32_t device_id) {

// 检查各个步骤的返回值，请勿在其他函数中调用
#define GX_CHECK_STATUS(status_code)                      \
    if ((status_code) != GX_STATUS_SUCCESS) {             \
    std::cout << GetErrorInfo(status_code) << std::endl;  \
    device_ = nullptr;                                    \
    return ERR_CAMERA_INTERNAL_ERROR;                     \
}

    GX_STATUS status_code;
    uint32_t device_num = 0;

    // 枚举设备
    status_code = GXUpdateDeviceList(&device_num, 1000);
    if (status_code != GX_STATUS_SUCCESS) {
        std::cout << GetErrorInfo(status_code) << std::endl;
    }

    // 检查是否有设备连接
    if (device_num <= 0) {
        return ERR_DEVICE_NOT_FOUND;
    }

    // 打开指定设备
    status_code = GXOpenDeviceByIndex(device_id, &device_);
    GX_CHECK_STATUS(status_code)

    // 检查是否能获取各项参数
    if (GetVendorName().empty() ||
        GetModelName().empty() ||
        GetSerialNumber().empty() ||
        GetDeviceVersion().empty()) {
        device_ = nullptr;
        return ERR_CAMERA_INTERNAL_ERROR;
    }

    // 判断是否是彩色相机
    bool has_color_filter = false;
    status_code = GXIsImplemented(device_, GX_ENUM_PIXEL_COLOR_FILTER, &has_color_filter);
    GX_CHECK_STATUS(status_code)

    // 此程序不支持黑白相机
    if (!has_color_filter) {
        std::cout << "{!}{Mono cameras are not supported}";
        device_ = nullptr;
        return ERR_UNSUPPORTED_CAMERA;
    } else {
        status_code = GXGetEnum(device_, GX_ENUM_PIXEL_COLOR_FILTER, &color_filter_);
        GX_CHECK_STATUS(status_code)
    }

    // 设置 Payload 大小
    status_code = GXGetInt(device_, GX_INT_PAYLOAD_SIZE, &payload_size_);
    GX_CHECK_STATUS(status_code)


    // 设置取图模式为连续
    status_code = GXSetEnum(device_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    GX_CHECK_STATUS(status_code)

    // 设置触发器关闭
    status_code = GXSetEnum(device_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    GX_CHECK_STATUS(status_code)

    // 设置缓冲区个数
    uint64_t buffer_num = ACQ_BUFFER_NUM;
    status_code = GXSetAcqusitionBufferNumber(device_, buffer_num);
    GX_CHECK_STATUS(status_code)

    // 设置数据流传输大小
    bool stream_transfer_size = false;
    status_code = GXIsImplemented(device_, GX_DS_INT_STREAM_TRANSFER_SIZE, &stream_transfer_size);
    GX_CHECK_STATUS(status_code)

    // 设置数据包大小
    if (stream_transfer_size) {
        status_code = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
        if (status_code != GX_STATUS_SUCCESS) {
            std::cout << GetErrorInfo(status_code) << std::endl;
            device_ = nullptr;
            return ERR_CAMERA_INTERNAL_ERROR;
        }
    }

    bool stream_transfer_number_urb = false;
    status_code = GXIsImplemented(device_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &stream_transfer_number_urb);
    GX_CHECK_STATUS(status_code)

    if (stream_transfer_number_urb) {
        status_code = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
        if (status_code != GX_STATUS_SUCCESS) {
            std::cout << GetErrorInfo(status_code) << std::endl;
            device_ = nullptr;
            return ERR_CAMERA_INTERNAL_ERROR;
        }
    }

    // 白平衡：自动
    status_code = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_ONCE);
    GX_CHECK_STATUS(status_code)

    // 自此之后，device_ 将不再变为空指针，直至关闭设备
    ++camera_number_;

    return SUCCESS;
}

DHCamera::ErrorCode DHCamera::CloseCamera() {
    if (thread_alive_) {
        StopAcquisition();
    }

    // 接下来无论如何 device_ 都会变成空指针
    --camera_number_;

    GX_STATUS status_code = GXCloseDevice(device_);
    if (status_code != GX_STATUS_SUCCESS) {
        std::cout << GetErrorInfo(status_code) << std::endl;
        device_ = nullptr;
        return ERR_CAMERA_INTERNAL_ERROR;
    }

    device_ = nullptr;

    if (!camera_number_) {
        status_code = GXCloseLib();
        if (status_code != GX_STATUS_SUCCESS) {
            std::cout << GetErrorInfo(status_code) << std::endl;
            return ERR_CAMERA_INTERNAL_ERROR;
        }
    }

    return SUCCESS;
}

DHCamera::ErrorCode DHCamera::StartAcquisition() {
    // 分配内存
    rgb_8_to_rgb_24_cache_ = new unsigned char[payload_size_ * 3];
    raw_16_to_8_cache_ = new unsigned char[payload_size_];

    // 开启数据流
    GX_STATUS status_code = GXStreamOn(device_);
    GX_CHECK_STATUS_WITH_STREAM(status_code)

    // 开启采集线程
    pthread_create(&thread_id_, nullptr, ThreadProc, this);

    return SUCCESS;
}

DHCamera::ErrorCode DHCamera::StopAcquisition() {
    thread_alive_ = false;

    pthread_join(thread_id_, nullptr);

    GX_STATUS status_code = GXStreamOff(device_);
    GX_CHECK_STATUS_WITH_STREAM(status_code)

    if (raw_16_to_8_cache_ != nullptr) {
        delete[] raw_16_to_8_cache_;
        raw_16_to_8_cache_ = nullptr;
    }
    if (rgb_8_to_rgb_24_cache_ != nullptr) {
        delete[] rgb_8_to_rgb_24_cache_;
        rgb_8_to_rgb_24_cache_ = nullptr;
    }

    return SUCCESS;
}

void *DHCamera::ThreadProc(void *obj) {
    GX_STATUS status_code;

    auto *self = (DHCamera *) obj;

    self->thread_alive_ = true;
    self->frame_buffer_ = nullptr;

    time_t start_time;
    time_t end_time;
    uint32_t frame_count = 0;

    while (self->thread_alive_) {
        if (!frame_count) {
            time(&start_time);
        }

        status_code = GXDQBuf(self->device_, &self->frame_buffer_, 1000);
        if (status_code != GX_STATUS_SUCCESS) {
            if (status_code == GX_STATUS_TIMEOUT) {
                continue;
            } else {
                std::cout << GetErrorInfo(status_code) << std::endl;
                break;
            }
        }

        if (self->frame_buffer_->nStatus != GX_FRAME_STATUS_SUCCESS) {
            std::cout << GetErrorInfo(self->frame_buffer_->nStatus) << std::endl;
        } else {
            frame_count++;
            time(&end_time);
            if (end_time - start_time >= 1) {
                printf("<Successful acquisition: FrameCount: %u Width: %d Height: %d FrameID: %lu>\n",
                       frame_count,
                       self->frame_buffer_->nWidth,
                       self->frame_buffer_->nHeight,
                       self->frame_buffer_->nFrameID);

                frame_count = 0;
            }
        }

        status_code = GXQBuf(self->device_, self->frame_buffer_);
        if (status_code != GX_STATUS_SUCCESS) {
            std::cout << GetErrorInfo(status_code) << std::endl;
            break;
        }
    }
    return nullptr;
}

std::string DHCamera::GetVendorName() {
    size_t str_size = 0;

    GX_STATUS status_code = GXGetStringLength(device_,
                                              GX_STRING_DEVICE_VENDOR_NAME,
                                              &str_size);
    if (status_code != GX_STATUS_SUCCESS) {
        std::cout << GetErrorInfo(status_code) << std::endl;
        return "";
    }

    char *vendor_name = new char[str_size];

    status_code = GXGetString(device_,
                              GX_STRING_DEVICE_VENDOR_NAME,
                              vendor_name,
                              &str_size);
    if (status_code != GX_STATUS_SUCCESS) {
        std::cout << GetErrorInfo(status_code) << std::endl;
        delete[] vendor_name;
        return "";
    }

    return vendor_name;
}

std::string DHCamera::GetModelName() {
    size_t str_size = 0;

    GX_STATUS status_code = GXGetStringLength(device_,
                                              GX_STRING_DEVICE_MODEL_NAME,
                                              &str_size);
    if (status_code != GX_STATUS_SUCCESS) {
        std::cout << GetErrorInfo(status_code) << std::endl;
        return "";
    }

    char *model_name = new char[str_size];

    status_code = GXGetString(device_,
                              GX_STRING_DEVICE_MODEL_NAME,
                              model_name,
                              &str_size);
    if (status_code != GX_STATUS_SUCCESS) {
        std::cout << GetErrorInfo(status_code) << std::endl;
        delete[] model_name;
        return "";
    }

    return model_name;
}

std::string DHCamera::GetSerialNumber() {
    size_t str_size = 0;

    GX_STATUS status_code = GXGetStringLength(device_,
                                              GX_STRING_DEVICE_SERIAL_NUMBER,
                                              &str_size);
    if (status_code != GX_STATUS_SUCCESS) {
        std::cout << GetErrorInfo(status_code) << std::endl;
        return "";
    }

    char *serial_number = new char[str_size];

    status_code = GXGetString(device_,
                              GX_STRING_DEVICE_SERIAL_NUMBER,
                              serial_number,
                              &str_size);
    if (status_code != GX_STATUS_SUCCESS) {
        std::cout << GetErrorInfo(status_code) << std::endl;
        delete[] serial_number;
        return "";
    }

    return serial_number;
}

std::string DHCamera::GetDeviceVersion() {
    size_t str_size = 0;

    GX_STATUS status_code = GXGetStringLength(device_,
                                              GX_STRING_DEVICE_VERSION,
                                              &str_size);
    if (status_code != GX_STATUS_SUCCESS) {
        std::cout << GetErrorInfo(status_code) << std::endl;
        return "";
    }

    char *device_version = new char[str_size];

    status_code = GXGetString(device_,
                              GX_STRING_DEVICE_VERSION,
                              device_version,
                              &str_size);
    if (status_code != GX_STATUS_SUCCESS) {
        delete[] device_version;
        std::cout << GetErrorInfo(status_code) << std::endl;
        return "";
    }

    return device_version;
}

std::string DHCamera::GetErrorInfo(GX_STATUS error_status_code) {
    size_t str_size = 0;

    if (GXGetLastError(&error_status_code,
                       nullptr,
                       &str_size) != GX_STATUS_SUCCESS) {
        return "{?}{Unknown error}";
    }

    char *error_info = new char[str_size];

    if (GXGetLastError(&error_status_code,
                       error_info,
                       &str_size) != GX_STATUS_SUCCESS) {
        delete[] error_info;
        return "{?}{Unknown error}";
    }

    return error_info;
}
