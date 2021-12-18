#include "dh_camera.h"
#include <unistd.h>

int main() {
    DHCamera cam = DHCamera();
    cam.OpenCamera();
    cam.StartAcquisition();
    sleep(10);
    cam.CloseCamera();
}
