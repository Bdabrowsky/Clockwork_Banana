#include <Arduino.h> 
#include "AHRS.h"
#include "Orientation.h"

void filterMagdwickUpdate(float w_x, float w_y, float w_z, float a_x,float a_y, float a_z, unsigned long deltaT, bool useAcc);
void AHRS_integrateVelocity(float deltaT, float ax, float ay, float az);
void AHRS_calculateRake();


AHRS_orientation_t AHRS_orientation_d;
AHRS_config_t AHRS_config_d;

Orientation ori; // Main orientation measurement
EulerAngles oriMeasure; 

void initAHRS(float pitchOffset, float yawOffset, float rollOffset, float gyroMeasError)
{
    AHRS_config_d.beta = sqrt(3.0f / 4.0f) * PI * (gyroMeasError / 180.0f);

    AHRS_config_d.pitchOffset=pitchOffset;
    AHRS_config_d.yawOffset=yawOffset;
    AHRS_config_d.rollOffset=rollOffset;


    AHRS_orientation_d.q1 = 1.0f;
    AHRS_orientation_d.q2 = 0.0f;
    AHRS_orientation_d.q3 = 0.0f;
    AHRS_orientation_d.q4 = 0.0f;
}

void IntegralSimpleupdate(float w_x, float w_y, float w_z, float deltaT)
{
    float norm = sqrtf(powf(w_z, 2) + powf(w_x, 2) + powf(w_y, 2));
    norm = copysignf(max(abs(norm), 1e-9), norm); 
    
    float q1_rotation = 1.0f,  q2_rotation = 0.0f, q3_rotation = 0.0F, q4_rotation = 0.0F;
    float sa = sin(deltaT * norm / 2);

    q1_rotation = cos(deltaT * norm / 2);
    q2_rotation = w_x / norm * sa;
    q3_rotation = w_y / norm * sa;
    q4_rotation = w_z / norm * sa;


    AHRS_orientation_d.q1 = AHRS_orientation_d.q1 * q1_rotation - AHRS_orientation_d.q2 * q2_rotation - AHRS_orientation_d.q3 * q3_rotation - AHRS_orientation_d.q4 * q4_rotation;
    AHRS_orientation_d.q2 = AHRS_orientation_d.q1 * q2_rotation + AHRS_orientation_d.q2 * q1_rotation + AHRS_orientation_d.q3 * q4_rotation - AHRS_orientation_d.q4 * q3_rotation;
    AHRS_orientation_d.q3 = AHRS_orientation_d.q1 * q3_rotation - AHRS_orientation_d.q2 * q4_rotation + AHRS_orientation_d.q3 * q1_rotation + AHRS_orientation_d.q4 * q2_rotation;
    AHRS_orientation_d.q4 = AHRS_orientation_d.q1 * q4_rotation + AHRS_orientation_d.q2*  q3_rotation - AHRS_orientation_d.q3 * q2_rotation + AHRS_orientation_d.q4 * q1_rotation;
}


void filterMagdwickUpdate(float w_x, float w_y, float w_z, float a_x,float a_y, float a_z, unsigned long deltaT, bool useAcc)
{
    float norm = 1;
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; 
    float f_1 , f_2, f_3;
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;
    // vector norm
    // quaternion derrivative from gyroscopes elements // objective function elements
    // objective function Jacobian elements
    // estimated direction of the gyroscope error
    float SEqHatDot_1 = 0, SEqHatDot_2 = 0,SEqHatDot_3 = 0, SEqHatDot_4 = 0;
    // Axulirary variables to avoid  reapeated calcualtions
    float halfSEq_1 = 0.5f * AHRS_orientation_d.q1;
    float halfSEq_2 = 0.5f * AHRS_orientation_d.q2;
    float halfSEq_3 = 0.5f * AHRS_orientation_d.q3; 
    float halfSEq_4 = 0.5f * AHRS_orientation_d.q4; 
    float twoSEq_1 = 2.0f * AHRS_orientation_d.q1; 
    float twoSEq_2 = 2.0f * AHRS_orientation_d.q2; 
    float twoSEq_3 = 2.0f * AHRS_orientation_d.q3;

    if(useAcc){
        // Normalise the accelerometer measurement
        norm = 1 / sqrt(a_x * a_x + a_y * a_y + a_z * a_z); 
        a_x *= norm;
        a_y *= norm;
        a_z *= norm;

        // Compute the objective function and Jacobian
        f_1 = twoSEq_2 * AHRS_orientation_d.q4 - twoSEq_1 * AHRS_orientation_d.q3 - a_x;
        f_2 = twoSEq_1 * AHRS_orientation_d.q2 + twoSEq_3 * AHRS_orientation_d.q4 - a_y;
        f_3 = 1.0f - twoSEq_2 * AHRS_orientation_d.q2 - twoSEq_3 * AHRS_orientation_d.q3 - a_z;
        J_11or24 = twoSEq_3;
        J_12or23 = 2.0f * AHRS_orientation_d.q4;
        J_13or22 = twoSEq_1;
        J_14or21 = twoSEq_2;
        J_32 = 2.0f * J_14or21;
        J_33 = 2.0f * J_11or24;
    
    
        // Compute the gradient (matrix multiplication)
        SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
        SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
        SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
        SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

        // J_11 negated in matrix multiplication
        // J_12 negated in matrix multiplication
        // negated in matrix multiplication // negated in matrix multiplication
        // Normalise the gradient

        norm = 1 / sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
        SEqHatDot_1 /= norm;
        SEqHatDot_2 *= norm;
        SEqHatDot_3 *= norm;
        SEqHatDot_4 *= norm;
    }
    

    // Compute the quaternion derrivative measured by gyroscopes 
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y; 
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;


    // Compute then integrate the estimated quaternion derrivative 
    AHRS_orientation_d.q1 += (SEqDot_omega_1 - (AHRS_config_d.beta * SEqHatDot_1)) * deltaT / 1000000.0;
    AHRS_orientation_d.q2 += (SEqDot_omega_2 - (AHRS_config_d.beta * SEqHatDot_2)) * deltaT / 1000000.0;
    AHRS_orientation_d.q3 += (SEqDot_omega_3 - (AHRS_config_d.beta * SEqHatDot_3)) * deltaT / 1000000.0;
    AHRS_orientation_d.q4 += (SEqDot_omega_4 - (AHRS_config_d.beta * SEqHatDot_4)) * deltaT / 1000000.0;

    

    // Normalise quaternion
    norm = 1 / sqrt(AHRS_orientation_d.q1 * AHRS_orientation_d.q1 + AHRS_orientation_d.q2 * AHRS_orientation_d.q2 + AHRS_orientation_d.q3 * AHRS_orientation_d.q3 + AHRS_orientation_d.q4 * AHRS_orientation_d.q4); 
    AHRS_orientation_d.q1 *= norm;
    AHRS_orientation_d.q2 *= norm;
    AHRS_orientation_d.q3 *= norm;
    AHRS_orientation_d.q4 *= norm; 

}

void AHRS_integrateVelocity(float deltaT, float ax, float ay, float az){
    AHRS_orientation_d.vx += ax * deltaT;
    AHRS_orientation_d.vy += ay * deltaT;
    AHRS_orientation_d.vz += az * deltaT;
}

void AHRS_calculateRake(){
    float Vq = sqrt(AHRS_orientation_d.vx * AHRS_orientation_d.vx + AHRS_orientation_d.vy * AHRS_orientation_d.vy + AHRS_orientation_d.vz * AHRS_orientation_d.vz);

    AHRS_orientation_d.rake = acos(AHRS_orientation_d.vz / Vq);
}

AHRS_orientation_t getPosition(int state, float deltaT, IMU_data_t imu)
{
   
    filterMagdwickUpdate(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az, deltaT, 0);
    
    //ori.update(imu.gx, imu.gy, imu.gz, deltaT / 1000000.0f); // '* DEG_TO_RAD' after all gyro functions if they return degrees/sec
    //oriMeasure = ori.toEuler();
    

    AHRS_orientation_d.yaw   = atan2(2.0f * (AHRS_orientation_d.q2 * AHRS_orientation_d.q3 + AHRS_orientation_d.q1 * AHRS_orientation_d.q4), AHRS_orientation_d.q1 * AHRS_orientation_d.q1 + AHRS_orientation_d.q2 * AHRS_orientation_d.q2 - AHRS_orientation_d.q3 * AHRS_orientation_d.q3 - AHRS_orientation_d.q4 * AHRS_orientation_d.q4);   
    AHRS_orientation_d.roll = -asin(2.0f * (AHRS_orientation_d.q2 * AHRS_orientation_d.q4 - AHRS_orientation_d.q1 * AHRS_orientation_d.q3));
    AHRS_orientation_d.pitch  = atan2(2.0f * (AHRS_orientation_d.q1 * AHRS_orientation_d.q2 + AHRS_orientation_d.q3 * AHRS_orientation_d.q4), AHRS_orientation_d.q1 * AHRS_orientation_d.q1 - AHRS_orientation_d.q2 * AHRS_orientation_d.q2 - AHRS_orientation_d.q3 * AHRS_orientation_d.q3 + AHRS_orientation_d.q4 * AHRS_orientation_d.q4);

    //AHRS_orientation_d.roll = oriMeasure.pitch;
    //AHRS_orientation_d.pitch = oriMeasure.yaw;
    //AHRS_orientation_d.yaw = oriMeasure.roll;

    float radToDeg=180.0f/PI;
    AHRS_orientation_d.pitch *= radToDeg;
    AHRS_orientation_d.roll *= radToDeg; 
    AHRS_orientation_d.yaw *= radToDeg;

    
    AHRS_orientation_d.pitch -= AHRS_config_d.pitchOffset;
    AHRS_orientation_d.yaw -= AHRS_config_d.yawOffset;
    AHRS_orientation_d.roll -= AHRS_config_d.rollOffset;

    //Serial.print(AHRS_orientation_d.pitch);Serial.print(" ");Serial.println(AHRS_orientation_d.yaw);

    return AHRS_orientation_d;
}