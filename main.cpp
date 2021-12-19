#include "dh_camera.h"
#include <iostream>
#include <ctime>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main() {
    DHCamera cam = DHCamera();
    cv::namedWindow("CAM", cv::WINDOW_AUTOSIZE);
    if (!cam.OpenCamera()) {
        std::cout << "Waiting for camera." << std::flush;
        while (!cam.OpenCamera()) {
            if (cv::waitKey(500) == 'q') {
                std::cout << " Aborted, exit program." << std::endl;
                return 0;
            }
            std::cout << "." << std::flush;
        }
        std::cout << " Connected." << std::endl;
    } else {
        std::cout << "Camera connected." << std::endl;
    }
    cam.SetFrameRate(60);
    cam.StartStream();
    cv::Mat img;
    uint32_t frame_count = 0;
    uint32_t last_second_frame_cont = 0;
    time_t start_time = time(nullptr), end_time;
    int pressed_key;
    for (uint32_t fps = 0; pressed_key != 'q'; ++frame_count) {
        if (!cam.GetImage(img)) {
            cam.CloseCamera();
            std::cout << "Camera disconnected unexpectedly. Reconnecting." << std::flush;
            while (!cam.OpenCamera()) {
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
            continue;
        }
        cv::putText(img,
                    std::to_string(frame_count) + " FPS: " + std::to_string(fps),
                    cv::Point(0, 24),
                    cv::FONT_HERSHEY_DUPLEX,
                    1,
                    cv::Scalar(0, 255, 0),
                    1,
                    false);
        cv::imshow("CAM", img);
        pressed_key = cv::waitKey(1);
        end_time = time(nullptr);
        if (end_time - start_time == 1) {
            fps = frame_count - last_second_frame_cont;
            last_second_frame_cont = frame_count;
            start_time = time(nullptr);
        } else if (end_time - start_time > 1) {
            last_second_frame_cont = frame_count;
            start_time = time(nullptr);
        }
    }
    img.release();
    cam.StopStream();
    cam.CloseCamera();
    return 0;
}
