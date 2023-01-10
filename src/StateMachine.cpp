#include "StateMachine.h"

 
StateMachine_data_t StateMachine_data_d;


void initStateMachine(float launchDetectionTreshold, float freefallAccelerationTreshold, float apogeeDetectionTreshold, float touchdownDetectionTreshold, float railHeight)
{
    StateMachine_data_d.lDT = launchDetectionTreshold;
    StateMachine_data_d.fAT = freefallAccelerationTreshold;
    StateMachine_data_d.aDT = apogeeDetectionTreshold;
    StateMachine_data_d.tDT=  touchdownDetectionTreshold;
    StateMachine_data_d.rH = railHeight;
}

bool liftoffDetection(StateMachine_state_t state, IMU_data_t imu, bool DEBUG_OUTPUT){ 
  if(state != GROUND_IDLE){
    return false;
  }
  
  if(imu.ay - 9.81 >= StateMachine_data_d.lDT * 9.81f)
  {
    if(DEBUG_OUTPUT){
      Serial.println("Launch detected!");
    }
  
    return true;
  }

  return false;
}  

bool burnoutDetection(StateMachine_state_t state, IMU_data_t imu, bool DEBUG_OUTPUT){
  if(state != POWERED_ASCENT){
    return false;
  }

  if(imu.ay < 0.0f){
    return true;
  }
  
  return false;
}

bool apogeeDetection(StateMachine_state_t state, IMU_data_t imu, BMP_data_t baro, bool DEBUG_OUTPUT){
  if(state != COAST){
    return false;
  }

  if( (baro.altitude + StateMachine_data_d.aDT <= baro.maxAltitude) && (baro.altitude > StateMachine_data_d.rH) ){
    return true;
  }

  return false;
}

bool touchdownDetection(StateMachine_state_t state, IMU_data_t imu, BMP_data_t baro, bool DEBUG_OUTPUT){
    if(state != MAIN_DESCENT){
        return false;
    }

    float at = sqrt(imu.ax*imu.ax + imu.ay*imu.ay + imu.az*imu.az);

    if(baro.altitude <= StateMachine_data_d.tDT && at >= 9.0f){
        return true;
    }
    
    return false;
}
 