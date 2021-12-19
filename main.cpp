#include "dh_camera.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main() {
    DHCamera cam = DHCamera();
    cv::namedWindow("CAM", cv::WINDOW_AUTOSIZE);
    std::cout << "Waiting for camera." << std::flush;
    while (!cam.OpenCamera()) {
        if (cv::waitKey(500) == 'q') {
            std::cout << " Aborted, exit program." << std::endl;
            return 0;
        }
        std::cout << "." << std::flush;
    }
    std::cout << " Connected." << std::endl;
    cam.SetFrameRate(60);
    cam.StartStream();
    cv::Mat img;
    for (uint32_t frame_count = 0; cv::waitKey(1) != 'q'; ++frame_count) {
        if (!cam.GetImage(img)) {
            cam.CloseCamera();
            std::cout << "Camera disconnected unexpectedly. Reconnecting." << std::flush;
            for (; !cam.OpenCamera();) {
                if (cv::waitKey(500) == 'q') {
                    std::cout << " Aborted, exit program." << std::endl;
                    img.release();
                    return 0;
                }
                std::cout << "." << std::flush;
            }
            std::cout << " Success." << std::endl;
            cam.SetFrameRate(60);
            cam.StartStream();
        }

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
    return 0;
}
