#pragma once

#include<Arduino.h>
#include "AHRS.h"
#include "Baro.h"
#include "IMU.h"
#include "StateMachine.h"
#include "Guidance_driver.h"


class pid{
    public:
        float kp;
        float ki;
        float kd;

        float previousErrorFiltered;
        float integral;
        float alpha;

        float maxOutput;
        float minOutput;


        float update(float value, float setpoint, float dT);
        float init(float kpI, float kiI, float kdI, float alphaI, float maxOutputI, float minOutputI);

};




