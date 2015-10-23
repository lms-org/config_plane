#ifndef HARDWARE_TO_PLANE_H
#define HARDWARE_TO_PLANE_H

#include <lms/datamanager.h>
#include <lms/module.h>
#include "sensor_utils/plane.h"
#include "sensor_utils/imu.h"
#include "sensor_utils/distance_sensor.h"
#include "imu_ahrs/madgwick_AHRS.h"

class HardwareToPlane : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
private:
    sensor_utils::Plane* plane;
    Madgwick ahrs;
    const sensor_utils::IMU* imu;
    sensor_utils::DistanceSensor* distanceSensor;
};

#endif // HARDWARE_TO_PLANE_H
