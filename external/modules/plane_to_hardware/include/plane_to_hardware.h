#ifndef PLANE_TO_HARDWARE_H
#define PLANE_TO_HARDWARE_H

#include <lms/datamanager.h>
#include <lms/module.h>

class PlaneToHardware : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
};

#endif // PLANE_TO_HARDWARE_H
