#ifndef _DH_CAMERA_H_
#define _DH_CAMERA_H_

#include <string>
#include <iostream>
#include "GxIAPI.h"
#include "DxImageProc.h"

/*
 * This macro is used to check if the device is successfully initialized.
 * !! DO NOT use this macro in other place !!
 */
#define GX_OPEN_CAMERA_CHECK_STATUS(status_code)          \
    if ((status_code) != GX_STATUS_SUCCESS) {             \
    std::cout << GetErrorInfo(status_code) << std::endl;  \
    device_ = nullptr;                                    \
    return false;                                         \
}

/*
 * This macro is used to check if parameters are successfully modified or set.
 * !! DO NOT use this macro in other place !!
 */
#define GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)    \
    if (status_code != GX_STATUS_SUCCESS) {               \
    std::cout << GetErrorInfo(status_code) << std::endl;  \
    return false;                                         \
}

/*
 * This macro is used to check if the stream is successfully opened or closed.
 * !! DO NOT use this macro in other place !!
 */
#define GX_START_STOP_ACQUISITION_CHECK_STATUS(status_code)  \
if ((status_code) != GX_STATUS_SUCCESS) {                    \
    if (raw_16_to_8_cache_ != nullptr) {                     \
        delete[] raw_16_to_8_cache_;                         \
        raw_16_to_8_cache_ = nullptr;                        \
    }                                                        \
    if (raw_8_to_rgb_24_cache_ != nullptr) {                 \
        delete[] raw_8_to_rgb_24_cache_;                     \
        raw_8_to_rgb_24_cache_ = nullptr;                    \
    }                                                        \
    std::cout << GetErrorInfo(status_code) << std::endl;     \
    return false;                                            \
}

class DHCamera {
public:
    DHCamera();

    DHCamera(const DHCamera &) = delete;

    DHCamera(const DHCamera &&) = delete;

    ~DHCamera();

    bool OpenCamera(uint32_t device_id = 1);

    bool StartAcquisition();

    bool StopAcquisition();

    bool CloseCamera();

    /*
     * GX_EXPOSURE_MODE_TIMED         = 1       => Control exposure time through exposure time register
     * GX_EXPOSURE_MODE_TRIGGERWIDTH  = 2       => Control exposure time through trigger signal width
     */
    inline bool SetExposureMode(const GX_EXPOSURE_MODE_ENTRY &p) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_, GX_ENUM_EXPOSURE_MODE, p);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        return true;
    }

    /*
     * GX_EXPOSURE_AUTO_OFF         = 0         => Switch off automatic exposure
     * GX_EXPOSURE_AUTO_CONTINUOUS  = 1         => Continuous automatic exposure
     * GX_EXPOSURE_AUTO_ONCE        = 2         => Single automatic exposure
     */
    inline bool SetExposureAuto(const GX_EXPOSURE_AUTO_ENTRY &p) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, p);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        return true;
    }

    /*
     * GX_EXPOSURE_TIME_MODE_ULTRASHORT  = 0    => Ultra short mode
     * GX_EXPOSURE_TIME_MODE_STANDARD    = 1    => Standard mode
     */
    inline bool SetExposureTimeMode(const GX_EXPOSURE_TIME_MODE_ENTRY &p) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_, GX_ENUM_EXPOSURE_TIME_MODE, p);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        return true;
    }

    inline bool SetExposureTime(int64_t t) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_, GX_FLOAT_EXPOSURE_TIME, t);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        return true;
    }

    /*
     * GX_GAIN_AUTO_OFF         = 0             => Switch off automatic gain
     * GX_GAIN_AUTO_CONTINUOUS  = 1             => Continuous automatic gain
     * GX_GAIN_AUTO_ONCE        = 2             => Single automatic gain
     */
    inline bool SetGainAuto(const GX_GAIN_AUTO_ENTRY &p) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_, GX_ENUM_GAIN_AUTO, p);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        return true;
    }

    inline bool SetGainValue(double gain) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)

        status_code = GXSetFloat(device_, GX_FLOAT_GAIN, (float) gain);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        double value = 0;
        status_code = GXGetFloat(device_, GX_FLOAT_GAIN, &value);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)

        return true;
    }

    /*
     * GX_BALANCE_WHITE_AUTO_ONCE
     * GX_BALANCE_WHITE_AUTO_CONTINUOUS
     */
    inline bool SetBalanceWhiteAuto(const GX_BALANCE_WHITE_AUTO_ENTRY &p) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_, GX_ENUM_GAIN_AUTO, p);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        return true;
    }

    /*
     * GX_TRIGGER_MODE_OFF  = 0                 => Switch off the trigger mode
     * GX_TRIGGER_MODE_ON   = 1                 => Switch on the trigger mode
     */
    inline bool SetTriggerMode(const GX_TRIGGER_MODE_ENTRY &p) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_, GX_ENUM_TRIGGER_MODE, p);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        return true;
    }

    /*
     * GX_TRIGGER_SOURCE_SOFTWARE     = 0       => Software trigger
     * GX_TRIGGER_SOURCE_LINE0        = 1       => Trigger source 0
     * GX_TRIGGER_SOURCE_LINE1        = 2       => Trigger source 1
     * GX_TRIGGER_SOURCE_LINE2        = 3       => Trigger source 2
     * GX_TRIGGER_SOURCE_LINE3        = 4       => Trigger source 3
     * GX_TRIGGER_SOURCE_COUNTER2END  = 5       => Counter 2 end trigger
     */
    inline bool SetTriggerSource(const GX_TRIGGER_SOURCE_ENTRY &p) {
        GX_STATUS status_code;
        status_code = GXSetEnum(device_, GX_ENUM_TRIGGER_SOURCE, p);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        return true;
    }

    // Payload size = RAW image size + Frame info size
    inline bool GetPayloadSize(int64_t *payload_size) {
        GX_STATUS status_code;
        status_code = GXGetInt(device_, GX_INT_PAYLOAD_SIZE, payload_size);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        return true;
    }

    /*
     * GX_COLOR_FILTER_NONE      = 0            => None
     * GX_COLOR_FILTER_BAYER_RG  = 1            => RG format
     * GX_COLOR_FILTER_BAYER_GB  = 2            => GB format
     * GX_COLOR_FILTER_BAYER_GR  = 3            => GR format
     * GX_COLOR_FILTER_BAYER_BG  = 4            => BG format
     */
    inline bool GetColorFilter(int64_t *color_filter) {
        GX_STATUS status_code;
        status_code = GXGetEnum(device_, GX_ENUM_PIXEL_COLOR_FILTER, color_filter);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        return true;
    }

    inline bool GetPixelFormat(int64_t *pixel_format) {
        GX_STATUS status_code;
        status_code = GXGetEnum(device_, GX_ENUM_PIXEL_FORMAT, pixel_format);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        return true;
    }

    inline bool SendTriggerCommand() {
        GX_STATUS status_code;
        status_code = GXSendCommand(device_, GX_COMMAND_TRIGGER_SOFTWARE);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        return true;
    }

    inline bool SetFrameRate(double fps) {
        GX_STATUS status_code;
        if (fps > 0) {
            status_code = GXSetEnum(device_,
                                    GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
                                    GX_ACQUISITION_FRAME_RATE_MODE_ON);
            GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)

            status_code = GXSetFloat(device_,
                                     GX_FLOAT_ACQUISITION_FRAME_RATE,
                                     fps);
            GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        } else {
            status_code = GXSetEnum(device_,
                                    GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
                                    GX_ACQUISITION_FRAME_RATE_MODE_OFF);
            GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        }
        return true;
    }

    inline bool RegisterCaptureCallback(GXCaptureCallBack callback) {
        GX_STATUS status_code;
        status_code = GXRegisterCaptureCallback(device_, this, callback);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        return true;
    }

    inline bool UnregisterCaptureCallback() {
        GX_STATUS status_code;
        status_code = GXUnregisterCaptureCallback(device_);
        GX_SET_GET_PARAMETER_CHECK_STATUS(status_code)
        return true;
    }

    inline GX_DEV_HANDLE GetDeviceHandle() { return device_; }

    std::string GetVendorName();

    std::string GetModelName();

    std::string GetSerialNumber();

    std::string GetDeviceVersion();

    static std::string GetErrorInfo(GX_STATUS);

    inline static unsigned int GetCameraNumber() { return camera_number_; }

private:
    static void *ThreadProc(void *);

    static unsigned int camera_number_;

    bool Raw8Raw16ToRGB24(PGX_FRAME_BUFFER);

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
