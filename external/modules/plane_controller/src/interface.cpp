#include "plane_controller.h"

extern "C" {
void* getInstance () {
    return new PlaneController();
}
}
