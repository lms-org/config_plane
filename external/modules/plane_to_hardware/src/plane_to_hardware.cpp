#include "plane_to_hardware.h"

bool PlaneToHardware::initialize() {
    datamanager()->readChannel<sensor_utils::Plane>(this,"PLANE");
    return true;
}

bool PlaneToHardware::deinitialize() {
    return true;
}

bool PlaneToHardware::cycle() {
    return true;
}
