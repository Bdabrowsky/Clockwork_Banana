#pragma once

#include<Arduino.h>
#include "AHRS.h"
#include "Baro.h"
#include "IMU.h"
#include "StateMachine.h"
#include "Guidance_driver.h"


typedef struct {
    float maxAngleX;
    float maxAngleY;
    float maxAngleZ;

    float Kp;
    float Ki;
    float Kd;
} PID_config_t;

typedef struct {
    float previousErrorX;
    float previousErrorY;
    float previousErrorZ;

    float sumErrorX;
    float sumErrorY;
    float sumErrorZ;

    float errorX;
    float errorY;
    float errorZ;

    
    
} PID_data_t;



void PID_init(double kp, double ki, double kd, float maxAngleX, float maxAngleY, float maxAngleZ);
Guidance_data_t PID_compute(AHRS_orientation_t ahrs, Guidance_data_t guidance, float timeStep);





