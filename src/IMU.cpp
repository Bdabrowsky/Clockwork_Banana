#include "IMU.h"
//#include "config.h"
#include <Adafruit_LSM6DSO32.h>

void calibrateGyro(int gyroCalibrationSampleCount);

IMU_config_t IMU_config_d;
IMU_data_t IMU_data_d;

Adafruit_LSM6DSO32 dso32;


IMU_data_t getRawReadings()
{ 
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  dso32.getEvent(&accel, &gyro, &temp);
   
  IMU_data_d.gx = gyro.gyro.x - IMU_config_d.gOX;
  IMU_data_d.gy = gyro.gyro.y - IMU_config_d.gOY;
  IMU_data_d.gz = gyro.gyro.z -IMU_config_d.gOZ;
    


    //High pass filter
    if(IMU_data_d.gx <= 0.03 && IMU_data_d.gx >= -0.03){
      IMU_data_d.gx = 0.0f;
    }

    if(IMU_data_d.gy <= 0.03 && IMU_data_d.gy >= -0.03){
      IMU_data_d.gy = 0.0f;
    }

    if(IMU_data_d.gz <= 0.03 && IMU_data_d.gz >= -0.03){
      IMU_data_d.gz = 0.0f;
    }

    
    IMU_data_d.ax = (-1.0f) * accel.acceleration.y;
    IMU_data_d.ay = (-1.0f) * accel.acceleration.x;
    IMU_data_d.az = (-1.0f) * accel.acceleration.z;
  

  return IMU_data_d;
}

void calibrateGyro(int gyroCalibrationSampleCount)
{
  double avgX=0,avgY=0,avgZ=0;

  //Take given number of samples 
  for(int i=0;i < gyroCalibrationSampleCount;i++){
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    dso32.getEvent(&accel, &gyro, &temp);

    avgX += gyro.gyro.x;
    avgY += gyro.gyro.y;
    avgZ += gyro.gyro.z;

    delay(20);
  }

    avgX /= (float)gyroCalibrationSampleCount;
    avgY /= (float)gyroCalibrationSampleCount;
    avgZ /= (float)gyroCalibrationSampleCount;

    IMU_config_d.gOX = avgX;
    IMU_config_d.gOY = avgY;
    IMU_config_d.gOZ = avgZ;

    Serial.println(avgX); Serial.println(avgY); Serial.println(avgZ);
}

bool initIMU(int gCSC)
{
  
    if(!dso32.begin_SPI(5))
    {
        if(1)
            Serial.println("Unable to initialize the IMU. Check your wiring!");
        return false;
      
    }

    dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
    dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
   

    if(1){
        Serial.println("Found LSM6DS0 9DOF");
    }

    calibrateGyro(gCSC);

    return true;
}
