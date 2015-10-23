#ifndef PLANE_CONTROLLER_H
#define PLANE_CONTROLLER_H

#include <lms/datamanager.h>
#include <lms/module.h>
#include "sensor_utils/plane.h"
#include "lms/math/vertex.h"
#include "sensor_utils/distance_sensor.h"
#include "sensor_utils/pid_controller.h"

class PlaneController : public lms::Module {
public:

    enum class PlaneState{
        IDLE,HOVER,FLY_MANUALLY
    };

    bool initialize();
    bool deinitialize();
    bool cycle();

private:
    sensor_utils::Plane* plane;
    PlaneState state;
    //hover stuff
    sensor_utils::PID hoverPidPitch;
    sensor_utils::PID hoverPidMotor;
    const sensor_utils::DistanceSensor* distanceToGround;
    const lms::ModuleConfig* config;

    void hover();
};

#endif // PLANE_CONTROLLER_H
