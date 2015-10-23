#include "hardware_to_plane.h"
#include "imu_ahrs/madgwick_AHRS.h"

bool HardwareToPlane::initialize() {
    plane = datamanager()->writeChannel<sensor_utils::Plane>(this,"PLANE");
    imu = datamanager()->readChannel<sensor_utils::IMU>(this,"IMU");
    distanceSensor = datamanager()->writeChannel<sensor_utils::DistanceSensor>(this,"DISTANCE");
    return true;
}

bool HardwareToPlane::deinitialize() {
    return true;
}

bool HardwareToPlane::cycle() {
    //TODO check if values are valid
    ahrs.update(imu->gyro.x,imu->gyro.y,imu->gyro.z,imu->acc.x,imu->acc.y,imu->acc.z,imu->magnetometer.x,imu->magnetometer.y,imu->magnetometer.z);

    plane->q0 = ahrs.q0();
    plane->q1 = ahrs.q1();
    plane->q2 = ahrs.q2();
    plane->q3 = ahrs.q3();
    return true;
}
