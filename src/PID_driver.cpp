#include "Pid_driver.h"

PID_data_t PID_data_d;
PID_config_t PID_config_d;


//X - pitch 
//Y - yaw
float pidX_P, pidY_P, 
pidX_I, pidY_I,
pidX_D, pidY_D;

float pidX, pidY;

void PID_init(double kp, double ki, double kd, float maxAngleX, float maxAngleY, float maxAngleZ){
    PID_config_d.Kp = kp;
    PID_config_d.Ki = ki;
    PID_config_d.Kd = kd;

    PID_config_d.maxAngleX = maxAngleX;
    PID_config_d.maxAngleX = maxAngleY;
}



Guidance_data_t PID_compute(AHRS_orientation_t ahrs, Guidance_data_t guidance, float timeStep){

    Guidance_data_t result = guidance;

    PID_data_d.previousErrorX = PID_data_d.errorX;
    PID_data_d.previousErrorY = PID_data_d.errorY;
    PID_data_d.previousErrorZ = PID_data_d.errorZ;


    //Obtain error 
    PID_data_d.errorX = ahrs.pitch - guidance.desiredPitch;
    PID_data_d.errorY = ahrs.yaw - guidance.desiredYaw;

    //Proportional part
    pidX_P = PID_data_d.errorX * PID_config_d.Kp;
    pidY_P = PID_data_d.errorY * PID_config_d.Kp;

    //Integral part
    PID_data_d.sumErrorX = PID_data_d.sumErrorX + PID_data_d.errorX * timeStep;
    PID_data_d.sumErrorY = PID_data_d.sumErrorY + PID_data_d.errorY * timeStep;

    pidX_I = PID_config_d.Ki * PID_data_d.sumErrorX;
    pidY_I = PID_config_d.Ki * PID_data_d.sumErrorY;
    
    //Derivative part
    pidX_D = PID_config_d.Kd * (PID_data_d.errorX - PID_data_d.previousErrorX) / timeStep;
    pidY_D = PID_config_d.Kd * (PID_data_d.errorY - PID_data_d.previousErrorY) / timeStep;


    //Sum all p i d components
    pidX = pidX_P + pidX_I + pidX_D;
    pidY = pidY_P + pidY_I + pidY_D;


    //Saturation filter
    if(pidX > PID_config_d.maxAngleX){
        pidX = PID_config_d.maxAngleX;
    }

    if(pidX < (-1) * PID_config_d.maxAngleX){
        pidX = (-1) * PID_config_d.maxAngleX;
    }

    if(pidY > PID_config_d.maxAngleY){
        pidY = PID_config_d.maxAngleY;
    }

    if(pidY > (-1) * PID_config_d.maxAngleY){
        pidY = (-1) * PID_config_d.maxAngleY;
    }
    

    result.servoValueX = pidX;
    result.servoValueY = pidY;

    return result;
}