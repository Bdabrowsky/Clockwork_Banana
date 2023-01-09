#pragma once

#include<Arduino.h>
#include "IMU.h"
#include "Baro.h"


typedef struct{
    float lDT;
    float fAT;
    float aDT;
    float tDT;
    float rH;
} StateMachine_data_t;

typedef enum{
    CRITICAL_ERROR,
    NON_CRITICAL_ERROR,
    FAILSAFE,
    INITIALIZING,
    GROUND_IDLE,
    POWERED_ASCENT,
    COAST,
    DROUGE_DESCENT,
    MAIN_DESCENT,
    TOUCHDOWN
} StateMachine_state_t;


void initStateMachine(float launchDetectionTreshold, float freefallAccelerationTreshold, float apogeeDetectionTreshold, float touchdownDetectionTreshold, float railHeight);

bool liftoffDetection(StateMachine_state_t state, IMU_data_t imu, bool DEBUG_OUTPUT);
bool burnoutDetection(StateMachine_state_t state, IMU_data_t imu, bool DEBUG_OUTPUT);
bool apogeeDetection(StateMachine_state_t state, IMU_data_t imu, BMP_data_t baro, bool DEBUG_OUTPUT);
bool touchdownDetection(StateMachine_state_t state, IMU_data_t imu, BMP_data_t baro, bool DEBUG_OUTPUT);



   
