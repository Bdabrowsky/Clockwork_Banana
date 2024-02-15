#include "StateMachine.h"

 
StateMachine_data_t StateMachine_data_d;

int cnt = 0;

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
  /*
  if( (baro.altitude + StateMachine_data_d.aDT <= baro.maxAltitude) && (baro.altitude > StateMachine_data_d.rH) ){
    return true;
  }
  */
 //Serial.print(baro.altitude);Serial.print(" ");Serial.println(baro.prevAltitude);
 
  /*if(floor(10.0*baro.altitude)/10.0 < floor(10.0*baro.prevAltitude)/10.0 ){
    cnt++;
  }
  else{
    cnt = 0;
  }
  //Serial.println(cnt);
  if(cnt == 100){
    return true;
  }
  */
  Serial.println(baro.pressure);
  if(baro.pressure > baro.prevPressure){
    cnt++;
  }
  else{
    cnt = 0;
  }
  //Serial.println(cnt);
  if(cnt == 100){
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
 