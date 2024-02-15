#pragma once

#include <Arduino.h>
#include <Wire.h>


typedef struct{
  float gOX;
  float gOY;
  float gOZ;
} IMU_config_t;

typedef struct{
  double ax;
  double ay;
  double az;

  double gx;
  double gy;
  double gz;
} IMU_data_t;



bool initIMU(int gCSC);
IMU_data_t getRawReadings();

      
 

