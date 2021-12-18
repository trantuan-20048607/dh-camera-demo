#include "dh_camera.h"
#include <unistd.h>
#include <iostream>

int main() {
    DHCamera cam = DHCamera();
    cam.OpenCamera();
    cam.StartAcquisition();
    std::cout << DHCamera::GetCameraNumber() << std::endl;
    sleep(10);
    cam.CloseCamera();
    std::cout << DHCamera::GetCameraNumber() << std::endl;
}
