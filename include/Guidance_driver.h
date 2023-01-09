#pragma once

#include "AHRS.h"
#include "Baro.h"
#include "IMU.h"
#include "StateMachine.h"

typedef struct{
    float desiredPitch;
    float desiredYaw;
    float desiredRoll;

    float servoValueX;
    float servoValueY;
    float servoValueZ;

} Guidance_data_t;