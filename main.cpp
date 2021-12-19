#include "dh_camera.h"
#include <unistd.h>

int main() {
    DHCamera cam = DHCamera();
    cam.OpenCamera();
    // VSync
    cam.SetFrameRate(60);
    cam.StartAcquisition();
    sleep(10);
    cam.CloseCamera();
}
