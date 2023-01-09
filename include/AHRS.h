#pragma once
 
#include "IMU.h"

typedef struct {
    float beta;  

    float pitchOffset;
    float yawOffset;
    float rollOffset;
    
} AHRS_config_t;

typedef struct {
    float q1; 
    float q2; 
    float q3;
    float q4;  

    float vx;
    float vy;
    float vz;

    float pitch;
    float yaw;
    float roll;

    float rake;
} AHRS_orientation_t;


void initAHRS(float pitchOffset, float yawOffset,float rollOffset, float gyroMeasError);
AHRS_orientation_t getPosition(int state, float deltaT, IMU_data_t imu);
   

