#include "hardware_to_plane.h"

extern "C" {
void* getInstance () {
    return new HardwareToPlane();
}
}
