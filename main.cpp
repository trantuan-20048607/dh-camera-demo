#include "dh_camera.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main() {
    DHCamera cam = DHCamera();
    cam.OpenCamera();
    cam.SetFrameRate(60);
    cam.StartStream();
    cv::Mat img;
    for (uint32_t frame_count = 0; cv::waitKey(1) != 'q'; ++frame_count) {
        cam.GetImage(img);
        cv::putText(img,
                    std::to_string(frame_count),
                    cv::Point(0, 24),
                    cv::FONT_HERSHEY_DUPLEX,
                    1,
                    cv::Scalar(0, 255, 0),
                    1,
                    false);
        cv::imshow("CAM", img);
    }
    img.release();
    cam.StopStream();
    cam.CloseCamera();
}
