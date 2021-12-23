#include <unistd.h>
#include <iostream>
#include <ctime>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "log/easylogging++.h"
#include "dh_camera.h"

int main() {
    DHCamera cam = DHCamera();

    el::Loggers::addFlag(el::LoggingFlag::ColoredTerminalOutput);
    el::Loggers::addFlag(el::LoggingFlag::MultiLoggerSupport);

    LOG(INFO) << "Waiting for camera...";

    while (!cam.OpenCamera("KE0210010102", "../config/KE0210010102.txt"))
        sleep(1);

    cam.SetFrameRate(60);
    cam.StartStream();
    cv::Mat img;
    uint32_t frame_count = 0;
    uint32_t last_second_frame_cont = 0;
    time_t start_time = time(nullptr), end_time;
    for (uint32_t fps = 0; cv::waitKey(10) != 'q'; ++frame_count) {
        if (!cam.GetImage(img)) {
            --frame_count;
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
