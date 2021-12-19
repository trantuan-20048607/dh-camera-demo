#include "dh_camera.h"
#include "GxIAPI.h"
#include "DxImageProc.h"
#include <string>
#include <iostream>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define ACQ_BUFFER_NUM          5             // Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE       (64 * 1024)   // Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64            // Qty. of data transfer block

// Number of objects with its device_ not nullptr.
unsigned int DHCamera::camera_number_ = 0;

DHCamera::DHCamera() {
    // Load external dynamic libraries.
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
}

bool DHCamera::OpenCamera(uint32_t device_id) {
    GX_STATUS status_code;
    uint32_t device_num = 0;

    // Get device list.
    status_code = GXUpdateDeviceList(&device_num, 1000);
    if (status_code != GX_STATUS_SUCCESS) {
        std::cout << GetErrorInfo(status_code) << std::endl;
    }

    // Find at least 1 device.
    if (device_num <= 0) {
        return false;
    }

    // Open the device of specified index.
    status_code = GXOpenDeviceByIndex(device_id, &device_);
    GX_OPEN_CAMERA_CHECK_STATUS(status_code)

    // Check if it's factory setting can be read.
    if (GetVendorName().empty() ||
        GetModelName().empty() ||
        GetSerialNumber().empty() ||
        GetDeviceVersion().empty()) {
        device_ = nullptr;
        return false;
    }

    // Check if it's mono or color camera.
    bool has_color_filter = false;
    status_code = GXIsImplemented(device_, GX_ENUM_PIXEL_COLOR_FILTER, &has_color_filter);
    GX_OPEN_CAMERA_CHECK_STATUS(status_code)

    // Mono cameras are NOT supported.
    if (!has_color_filter) {
        std::cout << "{!}{Mono cameras are NOT supported}";
        device_ = nullptr;
        return false;
    } else {
        status_code = GXGetEnum(device_, GX_ENUM_PIXEL_COLOR_FILTER, &color_filter_);
        GX_OPEN_CAMERA_CHECK_STATUS(status_code)
    }

    // Get payload size.
    status_code = GXGetInt(device_, GX_INT_PAYLOAD_SIZE, &payload_size_);
    GX_OPEN_CAMERA_CHECK_STATUS(status_code)

    // Set acquisition mode to continuous.
    status_code = GXSetEnum(device_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    GX_OPEN_CAMERA_CHECK_STATUS(status_code)

    // Turn off the trigger.
    status_code = GXSetEnum(device_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    GX_OPEN_CAMERA_CHECK_STATUS(status_code)

    // Set buffer number.
    uint64_t buffer_num = ACQ_BUFFER_NUM;
    status_code = GXSetAcqusitionBufferNumber(device_, buffer_num);
    GX_OPEN_CAMERA_CHECK_STATUS(status_code)

    // Set stream transfer size.
    bool stream_transfer_size = false;
    status_code = GXIsImplemented(device_, GX_DS_INT_STREAM_TRANSFER_SIZE, &stream_transfer_size);
    GX_OPEN_CAMERA_CHECK_STATUS(status_code)

    if (stream_transfer_size) {
        status_code = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
        if (status_code != GX_STATUS_SUCCESS) {
            std::cout << GetErrorInfo(status_code) << std::endl;
            device_ = nullptr;
            return false;
        }
    }

    bool stream_transfer_number_urb = false;
    status_code = GXIsImplemented(device_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &stream_transfer_number_urb);
    GX_OPEN_CAMERA_CHECK_STATUS(status_code)

    if (stream_transfer_number_urb) {
        status_code = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
        if (status_code != GX_STATUS_SUCCESS) {
            std::cout << GetErrorInfo(status_code) << std::endl;
            device_ = nullptr;
            return false;
        }
    }

    // White balance AUTO.
    status_code = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_ONCE);
    GX_OPEN_CAMERA_CHECK_STATUS(status_code)

    // device_ will not be nullptr until camera is closed, so camera_number_ plus 1.
    ++camera_number_;

    return true;
}

bool DHCamera::CloseCamera() {
    if (thread_alive_) {
        StopAcquisition();
    }

    // device_ must be nullptr next, so minus 1.
    --camera_number_;

    GX_STATUS status_code = GXCloseDevice(device_);
    if (status_code != GX_STATUS_SUCCESS) {
        std::cout << GetErrorInfo(status_code) << std::endl;
        device_ = nullptr;
        return false;
    }

    device_ = nullptr;

    if (!camera_number_) {
        status_code = GXCloseLib();
        if (status_code != GX_STATUS_SUCCESS) {
            std::cout << GetErrorInfo(status_code) << std::endl;
            return false;
        }
    }

    return true;
}

bool DHCamera::StartAcquisition() {
    // Allocate memory for cache.
    raw_8_to_rgb_24_cache_ = new unsigned char[payload_size_ * 3];
    raw_16_to_8_cache_ = new unsigned char[payload_size_];

    // Open the stream.
    GX_STATUS status_code = GXStreamOn(device_);
    GX_START_STOP_ACQUISITION_CHECK_STATUS(status_code)

    // Create C style thread.
    pthread_create(&thread_id_, nullptr, ThreadProc, this);

    return true;
}

bool DHCamera::StopAcquisition() {
    thread_alive_ = false;

    pthread_join(thread_id_, nullptr);

    // Close the stream.
    GX_STATUS status_code = GXStreamOff(device_);
    GX_START_STOP_ACQUISITION_CHECK_STATUS(status_code)

    // Release memory for cache.
    if (raw_16_to_8_cache_ != nullptr) {
        delete[] raw_16_to_8_cache_;
        raw_16_to_8_cache_ = nullptr;
    }
    if (raw_8_to_rgb_24_cache_ != nullptr) {
        delete[] raw_8_to_rgb_24_cache_;
        raw_8_to_rgb_24_cache_ = nullptr;
    }

    return true;
}

void *DHCamera::ThreadProc(void *obj) {
    GX_STATUS status_code;

    auto *self = (DHCamera *) obj;

    self->thread_alive_ = true;
    self->frame_buffer_ = nullptr;

    time_t start_time;
    time_t end_time;
    uint32_t frame_count = 0;

    time(&start_time);

    while (self->thread_alive_) {
        status_code = GXDQBuf(self->device_, &self->frame_buffer_, 1000);
        if (status_code != GX_STATUS_SUCCESS) {
            if (status_code == GX_STATUS_TIMEOUT) {
                continue;
            } else {
                std::cout << GetErrorInfo(status_code) << std::endl;
                break;
            }
        }

        status_code = GXQBuf(self->device_, self->frame_buffer_);
        if (status_code != GX_STATUS_SUCCESS) {
            std::cout << GetErrorInfo(status_code) << std::endl;
            break;
        }

        if (self->frame_buffer_->nStatus != GX_FRAME_STATUS_SUCCESS) {
            std::cout << GetErrorInfo(self->frame_buffer_->nStatus) << std::endl;
        } else {
            frame_count++;
            self->Raw8Raw16ToRGB24(self->frame_buffer_);
            cv::Mat mat = cv::Mat(self->frame_buffer_->nHeight,
                                  self->frame_buffer_->nWidth,
                                  CV_8UC3,
                                  self->raw_8_to_rgb_24_cache_);
            cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
            cv::putText(mat,
                        std::to_string(frame_count),
                        cv::Point(0, 24),
                        cv::FONT_HERSHEY_DUPLEX,
                        1,
                        cv::Scalar(0, 255, 0),
                        1,
                        false);
            cv::imshow("CAM", mat);
        }

        if ((cv::waitKey(5) & 0xff) == 'q') {
            self->StopAcquisition();
        }
    }

    time(&end_time);

    cv::destroyAllWindows();
    return nullptr;
}

bool DHCamera::Raw8Raw16ToRGB24(PGX_FRAME_BUFFER frame_buffer) {
    VxInt32 dx_status_code;

    // Convert RAW8 or RAW16 image to RGB24 image.
    switch (frame_buffer->nPixelFormat) {
        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_BG8: {
            // Convert to the RGB image.
            dx_status_code = DxRaw8toRGB24((unsigned char *) frame_buffer->pImgBuf,
                                           raw_8_to_rgb_24_cache_,
                                           frame_buffer->nWidth,
                                           frame_buffer->nHeight,
                                           RAW2RGB_NEIGHBOUR,
                                           DX_PIXEL_COLOR_FILTER(color_filter_),
                                           false);
            if (dx_status_code != DX_OK) {
                std::cout << "DxRaw8toRGB24 Failed, Error Code: " << dx_status_code << std::endl;
                return false;
            }
            break;
        }
        case GX_PIXEL_FORMAT_BAYER_GR10:
        case GX_PIXEL_FORMAT_BAYER_RG10:
        case GX_PIXEL_FORMAT_BAYER_GB10:
        case GX_PIXEL_FORMAT_BAYER_BG10:
        case GX_PIXEL_FORMAT_BAYER_GR12:
        case GX_PIXEL_FORMAT_BAYER_RG12:
        case GX_PIXEL_FORMAT_BAYER_GB12:
        case GX_PIXEL_FORMAT_BAYER_BG12: {
            // Convert to the Raw8 image.
            dx_status_code = DxRaw16toRaw8((unsigned char *) frame_buffer->pImgBuf,
                                           raw_16_to_8_cache_,
                                           frame_buffer->nWidth,
                                           frame_buffer->nHeight,
                                           DX_BIT_2_9);
            if (dx_status_code != DX_OK) {
                std::cout << "DxRaw16toRaw8 failed, error code: " << dx_status_code << std::endl;
                return false;
            }
            // Convert to the RGB24 image.
            dx_status_code = DxRaw8toRGB24(raw_16_to_8_cache_,
                                           raw_8_to_rgb_24_cache_,
                                           frame_buffer->nWidth,
                                           frame_buffer->nHeight,
                                           RAW2RGB_NEIGHBOUR,
                                           DX_PIXEL_COLOR_FILTER(color_filter_),
                                           false);
            if (dx_status_code != DX_OK) {
                std::cout << "DxRaw16toRGB24 failed, error code: " << dx_status_code << std::endl;
                return false;
            }
            break;
        }
        default: {
            std::cout << "Error: PixelFormat of this camera is not supported." << std::endl;
            return false;
        }
    }
    return true;
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
