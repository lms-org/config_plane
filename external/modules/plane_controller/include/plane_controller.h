#ifndef PLANE_CONTROLLER_H
#define PLANE_CONTROLLER_H

#include <lms/datamanager.h>
#include <lms/module.h>

class PlaneController : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
};

#endif // PLANE_CONTROLLER_H
