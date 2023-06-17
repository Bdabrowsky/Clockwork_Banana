#include "IMU.h"
#include <Adafruit_LSM6DS3.h>

void calibrateGyro(int gyroCalibrationSampleCount);

IMU_config_t IMU_config_d;
IMU_data_t IMU_data_d;

Adafruit_LSM6DS3 lsm6ds33;

IMU_data_t getRawReadings()
{ 
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
   
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

    
    IMU_data_d.ax = accel.acceleration.x;
    IMU_data_d.ay = accel.acceleration.y;
    IMU_data_d.az = accel.acceleration.z;
  

  return IMU_data_d;
}

void calibrateGyro(int gyroCalibrationSampleCount)
{
  double avgX,avgY,avgZ;

  //Take given number of samples 
  for(int i=0;i < gyroCalibrationSampleCount;i++){
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6ds33.getEvent(&accel, &gyro, &temp);

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

bool initIMU(int gCSC, bool DEBUG_OUTPUT)
{
  
    if(!lsm6ds33.begin_I2C(0x6A))
    {
        if(DEBUG_OUTPUT)
            Serial.println("Unable to initialize the IMU. Check your wiring!");
        return false;
      
    }

    lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    lsm6ds33.setAccelDataRate(LSM6DS_RATE_208_HZ);
    lsm6ds33.setGyroDataRate(LSM6DS_RATE_208_HZ);

    if(DEBUG_OUTPUT){
        Serial.println("Found LSM6DS0 9DOF");
    }

    calibrateGyro(gCSC);

    return true;
}
