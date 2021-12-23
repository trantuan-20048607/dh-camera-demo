#include <string>
#include <stdexcept>
#include <thread>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "GxIAPI.h"
#include "DxImageProc.h"

#include "dh_camera.h"
#include "log/easylogging++.h"

unsigned int DHCamera::camera_count_ = 0;

bool DHCamera::OpenCamera(const std::string &serial_number, const std::string &config_file) {
    // Will NOT check stream status for restart.
    // if (stream_running_) return false;
    if (device_ != nullptr) return false;

    // Load external dynamic libraries when no cameras were already opened.
    if (!camera_count_)
        if (GXInitLib() != GX_STATUS_SUCCESS)
            throw std::runtime_error("GX libraries not found.");

    GX_STATUS status_code;
    GX_OPEN_PARAM open_param;

    // Create an opening parameter structure.
    std::unique_ptr<char[]> sn(new char[1 + serial_number.size()]);
    strcpy(sn.get(), serial_number.c_str());

    open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    open_param.openMode = GX_OPEN_SN;
    open_param.pszContent = sn.get();

    // Try to open device.
    status_code = GXOpenDevice(&open_param, &device_);
    if (status_code != GX_STATUS_SUCCESS) {
        LOG(ERROR) << GetErrorInfo(status_code);
        device_ = nullptr;
        if (!camera_count_) {
            status_code = GXCloseLib();
            if (status_code != GX_STATUS_SUCCESS)
                LOG(ERROR) << GetErrorInfo(status_code);
        }
        return false;
    }

    serial_number_ = serial_number;

    // Check if it's mono or color camera.
    bool has_color_filter = false;
    status_code = GXIsImplemented(device_,
                                  GX_ENUM_PIXEL_COLOR_FILTER,
                                  &has_color_filter);
    GX_OPEN_CAMERA_CHECK_STATUS(status_code)

    // Mono cameras are NOT supported.
    if (!has_color_filter) {
        LOG(ERROR) << "{!}{Mono cameras are NOT supported}";
        status_code = GXCloseDevice(device_);
        if (status_code != GX_STATUS_SUCCESS)
            LOG(ERROR) << GetErrorInfo(status_code);
        device_ = nullptr;
        serial_number_ = "";
        if (!camera_count_) {
            status_code = GXCloseLib();
            if (status_code != GX_STATUS_SUCCESS)
                LOG(ERROR) << GetErrorInfo(status_code);
        }
        return false;
    } else {
        status_code = GXGetEnum(device_,
                                GX_ENUM_PIXEL_COLOR_FILTER,
                                &color_filter_);
        GX_OPEN_CAMERA_CHECK_STATUS(status_code)
    }

    // Get payload size.
    status_code = GXGetInt(device_,
                           GX_INT_PAYLOAD_SIZE,
                           &payload_size_);
    GX_OPEN_CAMERA_CHECK_STATUS(status_code)

    // Load configurations.
    if (!ImportConfigurationFile(config_file)) {
        status_code = GXCloseDevice(device_);
        if (status_code != GX_STATUS_SUCCESS)
            LOG(ERROR) << GetErrorInfo(status_code);
        device_ = nullptr;
        serial_number_ = "";
        if (!camera_count_) {
            status_code = GXCloseLib();
            if (status_code != GX_STATUS_SUCCESS)
                LOG(ERROR) << GetErrorInfo(status_code);
        }
        return false;
    }

    // device_ will not be nullptr until camera is closed, so camera_count_ plus 1.
    ++camera_count_;

    // Register callbacks.
    RegisterCaptureCallback(&DefaultCaptureCallback);

    // Start daemon thread.
    if (!daemon_thread_id_) {
        stop_daemon_thread_flag_ = false;
        pthread_create(&daemon_thread_id_, nullptr, DaemonThreadFunction, this);
        LOG(DEBUG) << serial_number_ << "'s daemon thread " << std::to_string(daemon_thread_id_) << " started.";
    }
    LOG(INFO) << "Opened camera " << serial_number_ << ".";
    return true;
}

bool DHCamera::CloseCamera() {
    if (stream_running_) return false;
    if (device_ == nullptr) return false;

    // Stop daemon thread.
    stop_daemon_thread_flag_ = true;
    pthread_join(daemon_thread_id_, nullptr);
    LOG(DEBUG) << serial_number_ << "'s daemon thread " << std::to_string(daemon_thread_id_) << " stopped.";

    // Reset daemon thread parameters.
    daemon_thread_id_ = 0;
    stop_daemon_thread_flag_ = false;

    // device_ must be nullptr next, so minus 1.
    --camera_count_;

    // Close device.
    GX_STATUS status_code = GXCloseDevice(device_);
    if (status_code != GX_STATUS_SUCCESS) {
        LOG(ERROR) << GetErrorInfo(status_code);

        device_ = nullptr;

        if (!camera_count_) {
            status_code = GXCloseLib();
            if (status_code != GX_STATUS_SUCCESS)
                LOG(ERROR) << GetErrorInfo(status_code);
        }

        return false;
    }

    device_ = nullptr;

    // Close libraries.
    if (!camera_count_) {
        status_code = GXCloseLib();
        if (status_code != GX_STATUS_SUCCESS) {
            LOG(ERROR) << GetErrorInfo(status_code);
            return false;
        }
    }

    LOG(INFO) << "Closed camera " << serial_number_ << ".";

    // Reset camera parameters.
    color_filter_ = GX_COLOR_FILTER_NONE;
    payload_size_ = 0;
    serial_number_ = "";
    return true;
}

bool DHCamera::StartStream() {
    if (device_ == nullptr) return false;
    if (stream_running_) return false;

    // Save temporary config to cache folder.
    ExportConfigurationFile("../cache/" + serial_number_ + ".txt");

    // Allocate memory for cache.
    raw_8_to_rgb_24_cache_ = new unsigned char[payload_size_ * 3];
    raw_16_to_8_cache_ = new unsigned char[payload_size_];

    // Open the stream.
    GX_STATUS status_code = GXStreamOn(device_);
    GX_START_STOP_STREAM_CHECK_STATUS(status_code)

    stream_running_ = true;
    LOG(INFO) << serial_number_ << "'s stream started.";
    return true;
}

bool DHCamera::StopStream() {
    if (device_ == nullptr) return false;
    if (!stream_running_) return false;

    stream_running_ = false;

    // Close the stream.
    GX_STATUS status_code = GXStreamOff(device_);
    GX_START_STOP_STREAM_CHECK_STATUS(status_code)

    // Release memory for cache.
    if (raw_16_to_8_cache_ != nullptr) {
        delete[] raw_16_to_8_cache_;
        raw_16_to_8_cache_ = nullptr;
    }
    if (raw_8_to_rgb_24_cache_ != nullptr) {
        delete[] raw_8_to_rgb_24_cache_;
        raw_8_to_rgb_24_cache_ = nullptr;
    }
    LOG(INFO) << serial_number_ << "'s stream stopped.";
    return true;
}

bool DHCamera::IsConnected() {
    if (device_ == nullptr) return false;

    GX_STATUS status_code;
    uint32_t device_num;

    // List all devices.
    status_code = GXUpdateDeviceList(&device_num, 1000);
    if (status_code != GX_STATUS_SUCCESS || device_num <= 0) {
        LOG(ERROR) << GetErrorInfo(status_code);
        return false;
    }

    // List all serial numbers.
    std::unique_ptr<GX_DEVICE_BASE_INFO[]> device_list(new GX_DEVICE_BASE_INFO[device_num]);
    size_t act_size = device_num * sizeof(GX_DEVICE_BASE_INFO);
    status_code = GXGetAllDeviceBaseInfo(device_list.get(), &act_size);
    if (status_code != GX_STATUS_SUCCESS) {
        LOG(ERROR) << GetErrorInfo(status_code);
        return false;
    }

    // Find known device.
    for (uint32_t i = 0; i < device_num; i++)
        if (device_list[i].szSN == this->serial_number_)
            return true;

    return false;
}

bool DHCamera::GetImage(cv::Mat &image) {
    if (!buffer_.Empty())
        image.release();
    return buffer_.Pop(image);
}

void DHCamera::DefaultCaptureCallback(GX_FRAME_CALLBACK_PARAM *param) {
    auto *camera = (DHCamera *) param->pUserParam;

    // Check image status.
    if (param->status != GX_STATUS_SUCCESS) {
        LOG(ERROR) << GetErrorInfo(param->status);
        return;
    }

    // Convert to RGB format.
    if (!camera->Raw8Raw16ToRGB24(param))
        return;

    // Generate OpenCV style image matrix.
    cv::Mat image = cv::Mat(param->nHeight,
                            param->nWidth,
                            CV_8UC3,
                            camera->raw_8_to_rgb_24_cache_);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    camera->buffer_[0].release();
    camera->buffer_.Push(image);
}

bool DHCamera::Raw8Raw16ToRGB24(GX_FRAME_CALLBACK_PARAM *frame_callback) {
    VxInt32 dx_status_code;

    // Convert RAW8 or RAW16 image to RGB24 image.
    switch (frame_callback->nPixelFormat) {
        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_BG8: {
            // Convert to the RGB image.
            dx_status_code = DxRaw8toRGB24((unsigned char *) frame_callback->pImgBuf,
                                           raw_8_to_rgb_24_cache_,
                                           frame_callback->nWidth,
                                           frame_callback->nHeight,
                                           RAW2RGB_NEIGHBOUR,
                                           DX_PIXEL_COLOR_FILTER(color_filter_),
                                           false);
            if (dx_status_code != DX_OK) {
                LOG(ERROR) << "DxRaw8toRGB24 failed with error " << std::to_string(dx_status_code) << ".";
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
            dx_status_code = DxRaw16toRaw8((unsigned char *) frame_callback->pImgBuf,
                                           raw_16_to_8_cache_,
                                           frame_callback->nWidth,
                                           frame_callback->nHeight,
                                           DX_BIT_2_9);
            if (dx_status_code != DX_OK) {
                LOG(ERROR) << "DxRaw8toRGB24 failed with error " << std::to_string(dx_status_code) << ".";
                return false;
            }
            // Convert to the RGB24 image.
            dx_status_code = DxRaw8toRGB24(raw_16_to_8_cache_,
                                           raw_8_to_rgb_24_cache_,
                                           frame_callback->nWidth,
                                           frame_callback->nHeight,
                                           RAW2RGB_NEIGHBOUR,
                                           DX_PIXEL_COLOR_FILTER(color_filter_),
                                           false);
            if (dx_status_code != DX_OK) {
                LOG(ERROR) << "DxRaw8toRGB24 failed with error " << std::to_string(dx_status_code) << ".";
                return false;
            }
            break;
        }
        default: {
            LOG(ERROR) << "Pixel format of this camera is not supported.";
            return false;
        }
    }
    return true;
}

void *DHCamera::DaemonThreadFunction(void *p) {
    auto self = (DHCamera *) p;

    while (!self->stop_daemon_thread_flag_) {
        sleep(1);
        if (!self->IsConnected()) {
            // Print information.
            LOG(ERROR) << self->serial_number_ << " is disconnected unexpectedly.";
            LOG(INFO) << "Preparing for reconnection...";

            // Stop the stream, errors are blocked.
            if (self->stream_running_) {
                GXStreamOff(self->device_);
                if (self->raw_16_to_8_cache_ != nullptr) {
                    delete[] self->raw_16_to_8_cache_;
                    self->raw_16_to_8_cache_ = nullptr;
                }
                if (self->raw_8_to_rgb_24_cache_ != nullptr) {
                    delete[] self->raw_8_to_rgb_24_cache_;
                    self->raw_8_to_rgb_24_cache_ = nullptr;
                }
            }

            // Unregister any callbacks.
            self->UnregisterCaptureCallback();

            // Close camera, errors are blocked.
            --camera_count_;
            GXCloseDevice(self->device_);

            // Clear cache except serial number and stream status.
            self->device_ = nullptr;
            self->payload_size_ = 0;
            self->color_filter_ = GX_COLOR_FILTER_NONE;
            self->daemon_thread_id_ = 0;

            // Clean libraries' cache
            if (!camera_count_)
                GXCloseLib();

            // Try reopening camera once per second, errors will be shown.
            while (!self->OpenCamera(self->serial_number_, "../cache/" + self->serial_number_ + ".txt"))
                sleep(1);

            // Restart Stream.
            if (self->stream_running_) {
                self->raw_8_to_rgb_24_cache_ = new unsigned char[self->payload_size_ * 3];
                self->raw_16_to_8_cache_ = new unsigned char[self->payload_size_];

                GX_STATUS status_code = GXStreamOn(self->device_);
                if (status_code != GX_STATUS_SUCCESS) {
                    LOG(ERROR) << GetErrorInfo(status_code);
                    GXStreamOff(self->device_);
                    if (self->raw_16_to_8_cache_ != nullptr) {
                        delete[] self->raw_16_to_8_cache_;
                        self->raw_16_to_8_cache_ = nullptr;
                    }
                    if (self->raw_8_to_rgb_24_cache_ != nullptr) {
                        delete[] self->raw_8_to_rgb_24_cache_;
                        self->raw_8_to_rgb_24_cache_ = nullptr;
                    }
                    self->stream_running_ = false;
                }
            }

            LOG(INFO) << self->serial_number_ << " is successfully reconnected.";
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
        LOG(ERROR) << GetErrorInfo(status_code);
        return "";
    }

    char *vendor_name = new char[str_size];

    status_code = GXGetString(device_,
                              GX_STRING_DEVICE_VENDOR_NAME,
                              vendor_name,
                              &str_size);
    if (status_code != GX_STATUS_SUCCESS) {
        LOG(ERROR) << GetErrorInfo(status_code);
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
        LOG(ERROR) << GetErrorInfo(status_code);
        return "";
    }

    char *model_name = new char[str_size];

    status_code = GXGetString(device_,
                              GX_STRING_DEVICE_MODEL_NAME,
                              model_name,
                              &str_size);
    if (status_code != GX_STATUS_SUCCESS) {
        LOG(ERROR) << GetErrorInfo(status_code);
        delete[] model_name;
        return "";
    }

    return model_name;
}

std::string DHCamera::GetDeviceVersion() {
    size_t str_size = 0;

    GX_STATUS status_code = GXGetStringLength(device_,
                                              GX_STRING_DEVICE_VERSION,
                                              &str_size);
    if (status_code != GX_STATUS_SUCCESS) {
        LOG(ERROR) << GetErrorInfo(status_code);
        return "";
    }

    char *device_version = new char[str_size];

    status_code = GXGetString(device_,
                              GX_STRING_DEVICE_VERSION,
                              device_version,
                              &str_size);
    if (status_code != GX_STATUS_SUCCESS) {
        delete[] device_version;
        LOG(ERROR) << GetErrorInfo(status_code);
        return "";
    }

    return device_version;
}
