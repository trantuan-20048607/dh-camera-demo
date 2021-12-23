#ifndef _DH_CAMERA_H_
#define _DH_CAMERA_H_

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "GxIAPI.h"
#include "DxImageProc.h"

#include "log/easylogging++.h"
#include "buffer.h"

#define GX_ACQ_BUFFER_NUM           5            // Acquisition Buffer Qty.
#define GX_ACQ_TRANSFER_SIZE        (64 * 1024)  // Size of data transfer block.
#define GX_ACQ_TRANSFER_NUMBER_URB  64           // Qty. of data transfer block.

/**
 * \brief This macro is used to check if the device is successfully initialized.
 * \attention !! DO NOT use this macro in other place !!
 */
#define GX_OPEN_CAMERA_CHECK_STATUS(status_code)          \
    if ((status_code) != GX_STATUS_SUCCESS) {             \
        LOG(ERROR) << GetErrorInfo(status_code);          \
        (status_code) = GXCloseDevice(device_);           \
        if ((status_code) != GX_STATUS_SUCCESS)           \
            LOG(ERROR) << GetErrorInfo(status_code);      \
        device_ = nullptr;                                \
        if (!camera_count_) {                             \
            (status_code) = GXCloseLib();                 \
            if ((status_code) != GX_STATUS_SUCCESS)       \
                LOG(ERROR) << GetErrorInfo(status_code);  \
        }                                                 \
    return false;                                         \
}

/**
 * \brief This macro is used to check if parameters are successfully modified or set.
 * \attention !! DO NOT use this macro in other place !!
 */
#define GX_CHECK_STATUS(status_code)           \
    if ((status_code) != GX_STATUS_SUCCESS) {  \
    LOG(ERROR) << GetErrorInfo(status_code);   \
    return false;                              \
}

/**
 * \brief This macro is used to check if the stream is successfully opened or closed.
 * \attention !! DO NOT use this macro in other place !!
 */
#define GX_START_STOP_STREAM_CHECK_STATUS(status_code)  \
if ((status_code) != GX_STATUS_SUCCESS) {               \
    if (raw_16_to_8_cache_ != nullptr) {                \
        delete[] raw_16_to_8_cache_;                    \
        raw_16_to_8_cache_ = nullptr;                   \
    }                                                   \
    if (raw_8_to_rgb_24_cache_ != nullptr) {            \
        delete[] raw_8_to_rgb_24_cache_;                \
        raw_8_to_rgb_24_cache_ = nullptr;               \
    }                                                   \
    LOG(ERROR) << GetErrorInfo(status_code);            \
    return false;                                       \
}

class DHCamera {
public:
    DHCamera() : device_(nullptr),
                 color_filter_(GX_COLOR_FILTER_NONE),
                 payload_size_(0),
                 stream_running_(false),
                 stop_daemon_thread_flag_(false),
                 raw_8_to_rgb_24_cache_(nullptr),
                 raw_16_to_8_cache_(nullptr),
                 daemon_thread_id_(0) {};

    DHCamera(const DHCamera &) = delete;

    DHCamera(DHCamera &&) = delete;

    DHCamera &operator=(const DHCamera &) = delete;

    DHCamera &operator=(const DHCamera &&) = delete;

    ~DHCamera() = default;

    /**
     * \brief Open a camera.
     * \param serial_number Serial number of the camera you wanna open.
     * \return A boolean shows whether the camera is successfully opened.
     */
    bool OpenCamera(const std::string &serial_number);

    /**
     * \brief Close the opened camera.
     * \return A boolean shows whether the camera is normally closed.
     * \attention No matter what is returned, the camera will not be valid.
     */
    bool CloseCamera();

    /**
     * \brief Start the stream.
     * \return Whether stream is started normally.
     * \attention This function will return false when stream is already started or camera is not opened.
     */
    bool StartStream();

    /**
     * \brief Stop the stream.
     * \return Whether stream is stopped normally.
     * \attention This function will return false when stream is not started or camera is not opened.
     */
    bool StopStream();

    /**
     * \brief Check if current device is connected.
     * \return A boolean shows current device is connected.
     * \attention
     *   Directly calling to this function will cost 1 sec.
     *   Thus, It should be put in a specialized thread.
     */
    bool IsConnected();

    /**
     * \brief Get an image from internal image buffer.
     * \param [out]image Acquired image will be stored here.
     * \return A boolean shows if buffer is not empty, or if you can successfully get an image.
     */
    bool GetImage(cv::Mat &);


    /**
     * \brief Export current config to specified file.
     * \param file_path File path.
     * \return A boolean shows if config is successfully saved.
     */
    inline bool ExportConfigurationFile(const std::string &file_path) {
        GX_STATUS status_code = GXExportConfigFile(device_, file_path.c_str());
        GX_CHECK_STATUS(status_code)

        LOG(INFO) << ("Saved " + serial_number_ + "'s configuration to " + file_path + ".");
        return true;
    }

    /**
     * \brief Import current config to specified file.
     * \param file_path File path.
     * \return A boolean shows if config is successfully imported.
     */
    inline bool ImportConfigurationFile(const std::string &file_path) {
        GX_STATUS status_code = GXImportConfigFile(device_, file_path.c_str());
        GX_CHECK_STATUS(status_code)

        LOG(INFO) << ("Imported configuration file " + file_path + " to " + serial_number_ + ".");
        return true;
    }

    [[maybe_unused]] inline bool SetFrameRate(double fps) {
        GX_STATUS status_code;
        if (fps > 0) {
            status_code = GXSetEnum(device_,
                                    GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
                                    GX_ACQUISITION_FRAME_RATE_MODE_ON);
            GX_CHECK_STATUS(status_code)

            status_code = GXSetFloat(device_,
                                     GX_FLOAT_ACQUISITION_FRAME_RATE,
                                     fps);
            GX_CHECK_STATUS(status_code)
        } else {
            status_code = GXSetEnum(device_,
                                    GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
                                    GX_ACQUISITION_FRAME_RATE_MODE_OFF);
            GX_CHECK_STATUS(status_code)
        }

        LOG(INFO) << (serial_number_ + "'s frame rate set to "
                      + (fps > 0 ? std::to_string(fps) : "unlimited") + ".");

        return true;
    }

    /*
     * Valid parameters:
     * GX_EXPOSURE_MODE_TIMED         = 1       => Control exposure time through exposure time register
     * GX_EXPOSURE_MODE_TRIGGERWIDTH  = 2       => Control exposure time through trigger signal width
     */
    [[maybe_unused]] inline bool SetExposureMode(const GX_EXPOSURE_MODE_ENTRY &gx_exposure_mode_entry) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_,
                                GX_ENUM_EXPOSURE_MODE,
                                gx_exposure_mode_entry);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << ("Set " + serial_number_ + "'s exposure mode to "
                       + std::to_string(gx_exposure_mode_entry) + ".");

        return true;
    }

    /*
     * Valid parameters:
     * GX_EXPOSURE_AUTO_OFF         = 0         => Switch off automatic exposure
     * GX_EXPOSURE_AUTO_CONTINUOUS  = 1         => Continuous automatic exposure
     * GX_EXPOSURE_AUTO_ONCE        = 2         => Single automatic exposure
     */
    [[maybe_unused]] inline bool SetExposureAuto(const GX_EXPOSURE_AUTO_ENTRY &gx_exposure_auto_entry) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_,
                                GX_ENUM_EXPOSURE_AUTO,
                                gx_exposure_auto_entry);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << ("Set " + serial_number_ + "'s exposure to auto mode "
                       + std::to_string(gx_exposure_auto_entry) + ".");

        return true;
    }

    /*
     * Valid parameters:
     * GX_EXPOSURE_TIME_MODE_ULTRASHORT  = 0    => Ultra short mode
     * GX_EXPOSURE_TIME_MODE_STANDARD    = 1    => Standard mode
     */
    [[maybe_unused]] inline bool SetExposureTimeMode(const GX_EXPOSURE_TIME_MODE_ENTRY &gx_exposure_time_mode_entry) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_,
                                GX_ENUM_EXPOSURE_TIME_MODE,
                                gx_exposure_time_mode_entry);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << ("Set " + serial_number_ + "'s exposure time mode to "
                       + std::to_string(gx_exposure_time_mode_entry) + ".");

        return true;
    }

    [[maybe_unused]] inline bool SetExposureTime(int64_t time) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_,
                                GX_FLOAT_EXPOSURE_TIME,
                                time);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << ("Set " + serial_number_ + "'s exposure time to "
                       + std::to_string(time) + ".");

        return true;
    }

    /*
     * Valid parameters:
     * GX_GAIN_AUTO_OFF         = 0             => Switch off automatic gain
     * GX_GAIN_AUTO_CONTINUOUS  = 1             => Continuous automatic gain
     * GX_GAIN_AUTO_ONCE        = 2             => Single automatic gain
     */
    [[maybe_unused]] inline bool SetGainAuto(const GX_GAIN_AUTO_ENTRY &gx_gain_auto_entry) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_,
                                GX_ENUM_GAIN_AUTO,
                                gx_gain_auto_entry);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << ("Set " + serial_number_ + "'s gain to auto mode "
                       + std::to_string(gx_gain_auto_entry) + ".");

        return true;
    }

    [[maybe_unused]] inline bool SetGainValue(double gain) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_,
                                GX_ENUM_GAIN_SELECTOR,
                                GX_GAIN_SELECTOR_ALL);
        GX_CHECK_STATUS(status_code)

        status_code = GXSetFloat(device_,
                                 GX_FLOAT_GAIN,
                                 (float) gain);
        GX_CHECK_STATUS(status_code)

        double value = 0;
        status_code = GXGetFloat(device_,
                                 GX_FLOAT_GAIN,
                                 &value);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << ("Set " + serial_number_ + "'s gain value to "
                       + std::to_string(gain) + ".");

        return true;
    }

    /*
     * Valid parameters:
     * GX_BALANCE_WHITE_AUTO_ONCE
     * GX_BALANCE_WHITE_AUTO_CONTINUOUS
     */
    [[maybe_unused]] inline bool SetBalanceWhiteAuto(const GX_BALANCE_WHITE_AUTO_ENTRY &gx_balance_white_auto_entry) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_,
                                GX_ENUM_GAIN_AUTO,
                                gx_balance_white_auto_entry);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << ("Set " + serial_number_ + "'s balance white to auto mode "
                       + std::to_string(gx_balance_white_auto_entry) + ".");

        return true;
    }

    /*
     * Valid parameters:
     * GX_TRIGGER_MODE_OFF  = 0                 => Switch off the trigger mode
     * GX_TRIGGER_MODE_ON   = 1                 => Switch on the trigger mode
     */
    [[maybe_unused]] inline bool SetTriggerMode(const GX_TRIGGER_MODE_ENTRY &gx_trigger_mode_entry) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_,
                                GX_ENUM_TRIGGER_MODE,
                                gx_trigger_mode_entry);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << ("Set " + serial_number_ + "'s trigger mode to "
                       + std::to_string(gx_trigger_mode_entry) + ".");

        return true;
    }

    /*
     * Valid parameters:
     * GX_TRIGGER_SOURCE_SOFTWARE     = 0       => Software trigger
     * GX_TRIGGER_SOURCE_LINE0        = 1       => Trigger source 0
     * GX_TRIGGER_SOURCE_LINE1        = 2       => Trigger source 1
     * GX_TRIGGER_SOURCE_LINE2        = 3       => Trigger source 2
     * GX_TRIGGER_SOURCE_LINE3        = 4       => Trigger source 3
     * GX_TRIGGER_SOURCE_COUNTER2END  = 5       => Counter 2 end trigger
     */
    [[maybe_unused]] inline bool SetTriggerSource(const GX_TRIGGER_SOURCE_ENTRY &gx_trigger_source_entry) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_,
                                GX_ENUM_TRIGGER_SOURCE,
                                gx_trigger_source_entry);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << ("Set " + serial_number_ + "'s trigger source to "
                       + std::to_string(gx_trigger_source_entry) + ".");

        return true;
    }

    // Payload size = RAW image size + Frame info size
    [[maybe_unused]] inline bool GetPayloadSize(int64_t *payload_size) {
        GX_STATUS status_code;
        status_code = GXGetInt(device_,
                               GX_INT_PAYLOAD_SIZE,
                               payload_size);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << (serial_number_ + "'s payload size is " + std::to_string(*payload_size) + ".");

        return true;
    }

    /*
     * Valid parameters:
     * GX_COLOR_FILTER_NONE      = 0            => None
     * GX_COLOR_FILTER_BAYER_RG  = 1            => RG format
     * GX_COLOR_FILTER_BAYER_GB  = 2            => GB format
     * GX_COLOR_FILTER_BAYER_GR  = 3            => GR format
     * GX_COLOR_FILTER_BAYER_BG  = 4            => BG format
     */
    [[maybe_unused]] inline bool GetColorFilter(int64_t *color_filter) {
        GX_STATUS status_code;
        status_code = GXGetEnum(device_,
                                GX_ENUM_PIXEL_COLOR_FILTER,
                                color_filter);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << (serial_number_ + "'s color filter is " + std::to_string(*color_filter) + ".");

        return true;
    }

    [[maybe_unused]] inline bool GetPixelFormat(int64_t *pixel_format) {
        GX_STATUS status_code;
        status_code = GXGetEnum(device_,
                                GX_ENUM_PIXEL_FORMAT,
                                pixel_format);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << (serial_number_ + "'s pixel format is " + std::to_string(*pixel_format) + ".");

        return true;
    }

    [[maybe_unused]] inline bool SendTriggerCommand() {
        GX_STATUS status_code;
        status_code = GXSendCommand(device_,
                                    GX_COMMAND_TRIGGER_SOFTWARE);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << ("Sent trigger command to " + serial_number_ + ".");

        return true;
    }


    [[maybe_unused]] inline bool RegisterCaptureCallback(GXCaptureCallBack callback) {
        GX_STATUS status_code;
        status_code = GXRegisterCaptureCallback(device_,
                                                this,
                                                callback);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << ("Registered " + serial_number_ + "'s capture callback.");

        return true;
    }

    [[maybe_unused]] inline bool UnregisterCaptureCallback() {
        GX_STATUS status_code;
        status_code = GXUnregisterCaptureCallback(device_);
        GX_CHECK_STATUS(status_code)

        LOG(DEBUG) << ("Unregistered " + serial_number_ + "'s capture callback.");

        return true;
    }


    [[maybe_unused]] inline GX_DEV_HANDLE GetDeviceHandle() { return device_; }

    [[maybe_unused]] std::string GetVendorName();

    [[maybe_unused]] std::string GetModelName();

    [[maybe_unused]] [[maybe_unused]] std::string GetSerialNumber();

    [[maybe_unused]] std::string GetDeviceVersion();


    [[maybe_unused]] inline static unsigned int GetCameraCount() { return camera_count_; }

private:
    /**
     * \brief Get error details for output.
     * \param error_status_code A status code returned from GxI API.
     * \return A string with error details.
     */
    [[maybe_unused]] inline static std::string GetErrorInfo(GX_STATUS error_status_code) {
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

    /**
     * \brief Convert RAW 8/16 pixel formats to an RGB 24 one.
     * \param frame_buffer A frame buffer acquired directly from camera.
     * \return A boolean shows whether pixel format is normally converted.
     */
    bool Raw8Raw16ToRGB24(GX_FRAME_CALLBACK_PARAM *);

    /**
     * \brief Internal capture callback function.
     * \param [in, out] param Camera itself.
     */
    static void GX_STDC DefaultCaptureCallback(GX_FRAME_CALLBACK_PARAM *);

    /**
     * \brief Daemon thread main function.
     * \attention !! Do NOT use this function in another place !!
     */
    static void *DaemonThreadFunction(void *);

    static unsigned int camera_count_;

    GX_DEV_HANDLE device_;

    bool stream_running_;

    bool stop_daemon_thread_flag_;
    pthread_t daemon_thread_id_;

    int64_t color_filter_;
    int64_t payload_size_;
    std::string serial_number_;

    unsigned char *raw_8_to_rgb_24_cache_;
    unsigned char *raw_16_to_8_cache_;

    Buffer<cv::Mat, 3> buffer_;
};

#endif  // _DH_CAMERA_H_
