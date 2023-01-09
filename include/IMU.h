#pragma once


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



bool initIMU(int gCSC, bool DEBUG_OUTPUT);
IMU_data_t getRawReadings();

      


