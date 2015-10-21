#include "plane_visualizer.h"

extern "C" {
void* getInstance () {
    return new PlaneVisualizer();
}
}
