#include "plane_to_hardware.h"

extern "C" {
void* getInstance () {
    return new PlaneToHardware();
}
}
