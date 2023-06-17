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
    PID_data_d.previousErrorZ = PID_data_d.errorZ;


    PID_data_d.errorZ = ahrs.pitch - guidance.desiredPitch;
    PID_data_d.errorX = ahrs.yaw - guidance.desiredYaw;
    //Serial.print(PID_data_d.errorZ);Serial.print(" ");Serial.println(PID_data_d.errorX);

    //Proportional part
    pidX_P = PID_data_d.errorX * PID_config_d.Kp;
    pidZ_P = PID_data_d.errorZ * PID_config_d.Kp;
    //Serial.print(pidX_P);Serial.print(" ");Serial.println(pidZ_P);


    //Integral part
    pidX_I = PID_config_d.Ki * (pidX_I + PID_data_d.errorX / timeStep);
    pidZ_I = PID_config_d.Ki * (pidZ_I + PID_data_d.errorZ / timeStep);
    //Serial.print(pidX_I);Serial.print(" ");Serial.println(pidZ_I);


    //Derivative part
    pidX_D = PID_config_d.Kd * (PID_data_d.errorX - PID_data_d.previousErrorX) / timeStep;
    pidZ_D = PID_config_d.Kd * (PID_data_d.errorZ - PID_data_d.previousErrorZ) / timeStep;
    //Serial.print(pidX_D);Serial.print(" ");Serial.println(pidZ_D);
    //Serial.println("");


    //Sum all p i d components
    pidX = pidX_P + pidX_I + pidX_D;
    pidZ = pidZ_P + pidZ_I + pidZ_D;

/*
    if(abs(pidX) > PID_config_d.maxAngleX){
        if(abs(pidX_P + pidX_D) <= PID_config_d.maxAngleX){
            float diff = PID_config_d.maxAngleX - abs(pidX_P + pidX_D);

            if(pidX > 0){
                pidX -= diff;   //angle positive -> subtract winded up integral part to jus hit saturation line
            }
            else{
                 pidX += diff;  //angle positive -> add winded up integral part to jus hit saturation line
            }
        }
    }
*/


/*
    //Saturation filter
    if(pidX > PID_config_d.maxAngleX){
        pidX = PID_config_d.maxAngleX;
    }

    if(pidX < (-1) * PID_config_d.maxAngleX){
        pidX = (-1) * PID_config_d.maxAngleX;
    }

    if(pidZ > PID_config_d.maxAngleZ){
        pidZ = PID_config_d.maxAngleZ;
    }

    if(pidZ < (-1) * PID_config_d.maxAngleZ){
        pidZ = (-1) * PID_config_d.maxAngleZ;
    }
*/
    result = guidance;
    result.servoValueX = pidX+90.0;
    result.servoValueZ = pidZ+90.0;

    Serial.print(pidX);Serial.print(" ");Serial.println(pidZ);
    return result;
}