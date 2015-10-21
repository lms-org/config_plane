#ifndef HARDWARE_TO_PLANE_H
#define HARDWARE_TO_PLANE_H

#include <lms/datamanager.h>
#include <lms/module.h>

class HardwareToPlane : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
};

#endif // HARDWARE_TO_PLANE_H
