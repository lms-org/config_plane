#ifndef PLANE_VISUALIZER_H
#define PLANE_VISUALIZER_H

#include <lms/datamanager.h>
#include <lms/module.h>

class PlaneVisualizer : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
};

#endif // PLANE_VISUALIZER_H
