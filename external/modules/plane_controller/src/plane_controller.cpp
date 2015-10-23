#include "plane_controller.h"

bool PlaneController::initialize() {
    plane = datamanager()->writeChannel<sensor_utils::Plane>(this,"PLANE");
    state = PlaneState::IDLE;

    distanceToGround = datamanager()->writeChannel<sensor_utils::DistanceSensor>(this,"DISTANCE");
    config = getConfig();
    hoverPidPitch.set(config->get<float>("Hover_Pitch_Bias",0),config->get<float>("Hover_Pitch_Kc",1),config->get<float>("Hover_Pitch_Ti",0),config->get<float>("Hover_Pitch_Td",0));

    hoverPidMotor.set(config->get<float>("Hover_Motor_Bias",0),config->get<float>("Hover_Motor_Kc",1),config->get<float>("Hover_Motor_Ti",0),config->get<float>("Hover_Motor_Td",0));
    return true;
}

bool PlaneController::deinitialize() {
    return true;
}

bool PlaneController::cycle() {
    //TODO
    switch (state) {
    case PlaneState::IDLE:
        //Idle :)
        break;
    case PlaneState::HOVER:
        hover();
        break;
    case PlaneState::FLY_MANUALLY:

        break;
    default:
        break;
    }
    return true;
}

void PlaneController::hover(){

    //float error = plane->theta;

    //TODO PID mit mehreren stellgrößen?
    //float res = hoverPid.pid(error);

    //calculate motor speed
    float targetHeigt = config->get<float>("targetHeight",1);

    float errorHeight = distanceToGround->distance-targetHeigt;
    float motorSpeed = hoverPidMotor.pid(errorHeight);
    plane->desiredMotorSpeed = motorSpeed;
}
