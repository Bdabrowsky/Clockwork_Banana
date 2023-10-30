#include "Pid_driver.h"


float pid::update(float value, float setpoint, float dT){
    
    //Calculate the error
    float error = setpoint - value;

    //Filter the error for derivative part
    float errorFiltered = alpha * error + (1.0f - alpha) * previousErrorFiltered;
    float derivative = (errorFiltered - previousErrorFiltered) / dT;

    float newIntegral = integral + error * dT;

    float output = kp * error + ki * integral + kd * derivative;


    //Saturation filter
    if(output > maxOutput){
        output = maxOutput;
    }
    else if(output < minOutput){
        output = minOutput;
    }
    else{
        //Anti windup
        integral = newIntegral;
    }
   
    return output; 
}

float pid::init(float kpI, float kiI, float kdI, float alphaI, float maxOutputI, float minOutputI){
    kp = kpI;
    ki = kiI;
    kd = kdI;

    alpha = alphaI;
    maxOutput = maxOutputI;
    minOutput = minOutputI;

}