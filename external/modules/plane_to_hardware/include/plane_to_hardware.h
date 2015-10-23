#ifndef PLANE_TO_HARDWARE_H
#define PLANE_TO_HARDWARE_H

#include <lms/datamanager.h>
#include <lms/module.h>

#include "sensor_utils/plane.h"

class PlaneToHardware : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();

private:
    const sensor_utils::Plane* plane;
};

#endif // PLANE_TO_HARDWARE_H
