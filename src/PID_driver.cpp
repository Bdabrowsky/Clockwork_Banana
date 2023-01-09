#include "Pid_driver.h"

PID_data_t PID_data_d;
PID_config_t PID_config_d;

float pidX_P, pidY_P, pidZ_P, 
pidX_I, pidY_I, pidZ_I, 
pidX_D, pidY_D, pidZ_D;

float pidX, pidY, pidZ;

void PID_init(double kp, double ki, double kd, float maxAngleX, float maxAngleY, float maxAngleZ){
    PID_config_d.Kp = kp;
    PID_config_d.Ki = ki;
    PID_config_d.Kd = kd;

    PID_config_d.maxAngleX = maxAngleX;
    PID_config_d.maxAngleX = maxAngleY;
    PID_config_d.maxAngleX = maxAngleZ;
}



Guidance_data_t PID_compute(AHRS_orientation_t ahrs, Guidance_data_t guidance, float timeStep){

    Guidance_data_t result;

    PID_data_d.previousErrorX = PID_data_d.errorX;
    PID_data_d.previousErrorY = PID_data_d.errorY;
    PID_data_d.previousErrorZ = PID_data_d.errorZ;

    PID_data_d.errorX = ahrs.pitch - guidance.desiredPitch;
    PID_data_d.errorY = ahrs.yaw - guidance.desiredYaw;
    PID_data_d.errorZ = ahrs.roll - guidance.desiredRoll;

    //Proportional part
    pidX_P = PID_data_d.errorX * PID_config_d.Kp;
    pidZ_P = PID_data_d.errorZ * PID_config_d.Kp;

    //Integral part
    pidX_I = PID_config_d.Ki * (pidX_I + PID_data_d.errorX / timeStep);
    pidZ_I = PID_config_d.Ki * (pidZ_I + PID_data_d.errorZ / timeStep);

    //Derivative part
    pidX_D = PID_config_d.Kd * (PID_data_d.errorX - PID_data_d.previousErrorX) / timeStep;
    pidZ_D = PID_config_d.Kd * (PID_data_d.errorZ - PID_data_d.previousErrorY) / timeStep;


    //Sum all p i d components
    pidX = pidX_P + pidX_I + pidX_D;
    pidZ = pidZ_P + pidZ_I + pidZ_D;


    //Saturation filter
    if(pidX > PID_config_d.maxAngleX){
        pidX = PID_config_d.maxAngleX;
    }

    if(pidY > PID_config_d.maxAngleY){
        pidY = PID_config_d.maxAngleY;
    }

    if(pidZ > PID_config_d.maxAngleZ){
        pidZ = PID_config_d.maxAngleZ;
    }


    result = guidance;
    result.servoValueX = pidX;
    result.servoValueX = pidZ;

    return result;
}