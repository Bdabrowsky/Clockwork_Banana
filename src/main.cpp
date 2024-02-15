#include<Arduino.h>
#include <Wire.h>
#include <String.h>

#include "config.h"
#include "Baro.h"
#include "IMU.h"
#include "StateMachine.h"



//Global
StateMachine_state_t StateMachine_state;
unsigned long prevTMain,lastUpdate,prevTLog,prevTBuzz;
unsigned long timestamp;

String packet;
int logCounter = 0;

unsigned long prevTDebug;

BMP_data_t BMP_data;
IMU_data_t IMU_data;


void setup(){
    pinMode(Buzzer, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(P1_EN, OUTPUT);
    pinMode(P2_EN, OUTPUT);

    StateMachine_state = INITIALIZING;

    if(DEBUG_OUTPUT){
        Serial.begin(9600);
    }

    
    if(!initBaro(DEBUG_OUTPUT) && StateMachine_state == INITIALIZING){
        StateMachine_state = CRITICAL_ERROR;
        Serial.println("BMP280 init failed!");
    }

    /*if(!SD_init(SDPin, DEBUG_OUTPUT) && StateMachine_state == INITIALIZING){
        StateMachine_state = CRITICAL_ERROR;
        Serial.println("SD init failed!");
    }*/
    
    initStateMachine(launchDetectionTreshold, freefallAccelerationTreshold, apogeeDetectionTreshold, touchdownDetectionTreshold, railHeight);

    if(!initIMU(gyroCalibrationSampleCount) && StateMachine_state == INITIALIZING){
        StateMachine_state = CRITICAL_ERROR;
        Serial.println("IMU init failed!");
    }

    if(StateMachine_state != CRITICAL_ERROR){
        StateMachine_state = GROUND_IDLE;
        Serial.println("Init successful !");
    }
   
}


void loop(){
    
   

    if( (micros()-prevTMain) >= (1000000./mainLoopFREQ) ){
        prevTMain=micros();
        
        IMU_data = getRawReadings();
        BMP_data = getAltitude();
        

      
        timestamp = micros(); //Get new microsecond timestamp for this loop
        unsigned long deltaT = timestamp - lastUpdate; 
        lastUpdate = timestamp;

        

        //State machine
        if(StateMachine_state == CRITICAL_ERROR){
            //Serial.println("ERROR");
        }
        else if(StateMachine_state == NON_CRITICAL_ERROR){
        
        }
        else if(StateMachine_state == GROUND_IDLE){
            if(liftoffDetection(StateMachine_state, IMU_data, DEBUG_OUTPUT)){
                StateMachine_state = POWERED_ASCENT;
                dataLoggingFREQ=25;
            }
        }
        else if(StateMachine_state == POWERED_ASCENT){
            if(burnoutDetection(StateMachine_state, IMU_data, DEBUG_OUTPUT)){
                StateMachine_state = COAST;
            }
        }
        else if(StateMachine_state == COAST){
            if(apogeeDetection(StateMachine_state, IMU_data, BMP_data, DEBUG_OUTPUT)){
                digitalWrite(P2_EN, HIGH);
                delay(300);
                digitalWrite(P2_EN, LOW);

                StateMachine_state = DROUGE_DESCENT;
                dataLoggingFREQ=10;
            }
        }
        else if(StateMachine_state == DROUGE_DESCENT){
            if(BMP_data.altitude <= mainAltitude){
                digitalWrite(P1_EN, HIGH);
                delay(300);
                digitalWrite(P1_EN, LOW);
                
                StateMachine_state = MAIN_DESCENT;
            }
        
        }
        else if(StateMachine_state == MAIN_DESCENT){
            if(touchdownDetection(StateMachine_state, IMU_data, BMP_data, DEBUG_OUTPUT)){
                StateMachine_state = TOUCHDOWN;
            }
        }
        else if(StateMachine_state == TOUCHDOWN){
            //SD_close();
        }
       
    }

    
    if( (micros()-prevTLog) >= (1000000./dataLoggingFREQ) && StateMachine_state != TOUCHDOWN){
        prevTLog = micros();

        String frame;
        frame = String(micros()) + String(",") + String(StateMachine_state) + String(",") 
        + String(IMU_data.ax) + String(",") + String(IMU_data.ay) + String(",") + String(IMU_data.az) + String(",") 
        + String(IMU_data.gx) + String(",") + String(IMU_data.gy) + String(",") + String(IMU_data.gz) + String(",") 
        + String(BMP_data.pressure) + String(",") + String(BMP_data.altitude) + String(",") ;


        packet += frame;

        logCounter++;
        if(logCounter >=50){
            //SD_writeData(packet);
            packet = "";
            logCounter = 0;
        }
       //Serial.println(frame);
    }
   
    if( (micros()-prevTDebug) >= (1000000./debugFREQ) && DEBUG_OUTPUT){
        prevTDebug = micros();
        Serial.println(BMP_data.altitude);
        //Serial.print(IMU_data.ax);Serial.print(" ");Serial.print(IMU_data.ay);Serial.print(" ");Serial.print(IMU_data.az);Serial.print(" ");Serial.print(BMP_data.altitude);Serial.print(" ");Serial.println(StateMachine_state);
       //Serial.print(AHRS_orientation.pitch);Serial.print(" ");Serial.print(AHRS_orientation.yaw);Serial.print(" ");Serial.println(AHRS_orientation.roll);Serial.print(" ");Serial.print(Guidance_data.servoValueX);Serial.print(" ");Serial.println(Guidance_data.servoValueZ);
        //Serial.print(IMU_data.gx);Serial.print(" ");Serial.println(IMU_data.ax);
    }

    

}