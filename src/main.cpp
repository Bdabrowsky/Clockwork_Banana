#include<Arduino.h>
#include <Wire.h>
#include <String.h>

#include "AHRS.h"
#include "Baro.h"
#include "IMU.h"
#include "StateMachine.h"
#include "SD_driver.h"
#include "PID_driver.h"
#include "Guidance_driver.h"
#include "Servo_driver.h"


//Board config
const int buzzerPin=                                    4;                          //Buzzer attachment pin
const int SDPin=                                        9;
const int S1Pin=                                        5;
const int S2Pin=                                        6;
const int S3Pin=                                        7;
const int S4Pin=                                        8;

//Tuning parameters, DO NOT TOUCH ANYTHING ELSE!
bool            DEBUG_OUTPUT=                           true;                       //Choose if computer should work in debug mode, printing logs on serial

//Guidance settings
float           desiredPitch=                           0;                          //Angle of stabilization in degrees
float           desiredYaw=                             0;
float           desiredRoll=                            0;

//Gyroscope settings
const int             gyroCalibrationSampleCount=       700;                        //Amount of samples taken for gyroscope calibration. 1000 is about 5-10 seconds of calibration
const float           gyroMeasError=                    20.0f;  

const float           pitchOffset=                      90.0f;                           //Offsets of rocket and IMU orientations in deg
const float           yawOffset=                        0;
const float           rollOffset=                       0;    

//Barometer settings
const int             altitudeAverageCnt=               10;                         //Number of measurments used in altitude averaging

//State machine settings
const float           launchDetectionTreshold=          1.2;                         //Launch trigger acceleration in g
const int             pyroTriggerTime=                  100;                        //Pyro ON time in miliseconds
const float           apogeeDetectionTreshold=          2;                          //Altitude difference in meters between max reading and actual reading able to trigger apogee detection
const float           touchdownDetectionTreshold=       10;                         //Altitude at which controller assumes it landed
const float           freefallAccelerationTreshold=     1;                          //Acceleration at which computer assumse it's frefallling in m/s^2
const unsigned long   stabilizationDelay=               0;                          //Delay between liftoff and stabilization start in seconds
const float           maxAngle=                         60.0f;                      //Maximum angle before flight termination
const float           railHeight=                       2;

//Refresh rate settings
int                   dataLoggingFREQ=                  2;                          //SD data logging frequency during ascent in HZ, on the ground 1/4 of that
unsigned long         mainLoopFREQ=                     150;                         //Main logic loop frequency in HZ
int                   buzzerFREQ=                       1;                          //frequency of buzzing during ground operation, 2 times thst after liftoff, 4 times that on apogee, 32 on error
int                   debugFREQ=                        5; 

//Pid tuning parameters
const float           kp=                               0.55;                      
const float           ki=                               0.08;    
const float           kd=                               0.28;   

const float           maxAngleX=                        30;
const float           maxAngleY=                        30;
const float           maxAngleZ=                        30;






//Global
StateMachine_state_t StateMachine_state;
unsigned long prevTMain,lastUpdate,prevTLog,prevTBuzz;
unsigned long timestamp;

String packet;
int logCounter = 0;

unsigned long prevTDebug;


Guidance_data_t Guidance_data;
AHRS_orientation_t AHRS_orientation;
BMP_data_t BMP_data;
IMU_data_t IMU_data;


void setup(){
    pinMode(buzzerPin, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    StateMachine_state = INITIALIZING;

    if(DEBUG_OUTPUT){
       
    }
    delay(5000);
    if(!initBaro(DEBUG_OUTPUT) && StateMachine_state == INITIALIZING){
        StateMachine_state = CRITICAL_ERROR;
        Serial.println("BMP280 init failed!");
    }

    /*if(!SD_init(SDPin, DEBUG_OUTPUT) && StateMachine_state == INITIALIZING){
        StateMachine_state = CRITICAL_ERROR;
        Serial.println("SD init failed!");
    }*/
    
    initAHRS(pitchOffset,yawOffset,rollOffset,gyroMeasError);
    PID_init(kp, ki, kd, maxAngleX, maxAngleX, maxAngleX);
    Servo_init(S1Pin, S2Pin, S3Pin, S4Pin);
    initStateMachine(launchDetectionTreshold, freefallAccelerationTreshold, apogeeDetectionTreshold, touchdownDetectionTreshold, railHeight);

    if(!initIMU(gyroCalibrationSampleCount, DEBUG_OUTPUT) && StateMachine_state == INITIALIZING){
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

        AHRS_orientation = getPosition(StateMachine_state, deltaT, IMU_data);
        //Serial.println(deltaT/1000);

        //Guidance
    
            Guidance_data = PID_compute(AHRS_orientation, Guidance_data, deltaT);

            Servo_execute(Guidance_data.servoValueX, Guidance_data.servoValueZ, -1*Guidance_data.servoValueX +180, -1*Guidance_data.servoValueZ+180);
        

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
                StateMachine_state = DROUGE_DESCENT;
                dataLoggingFREQ=10;
            }
        }
        else if(StateMachine_state == DROUGE_DESCENT){
        
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
        + String(BMP_data.pressure) + String(",") + String(BMP_data.altitude) + String(",") 
        + String(AHRS_orientation.pitch) + String(",") + String(AHRS_orientation.yaw) + String(",") + String(AHRS_orientation.roll) + String(",")
        + String(Guidance_data.desiredPitch) + String(",") + String(Guidance_data.desiredYaw) + String(",") + String(Guidance_data.desiredRoll) + String("\n");



        packet += frame;

        logCounter++;
        if(logCounter >=50){
            //SD_writeData(packet);
            packet = "";
            logCounter = 0;
        }
       //Serial.println(frame);
    }
   
    if( (micros()-prevTDebug) >= (1000000./debugFREQ)){
        prevTDebug = micros();
        //Serial.print(IMU_data.gx);Serial.print(" ");Serial.print(IMU_data.gy);Serial.print(" ");Serial.println(IMU_data.gz);
       //Serial.print(AHRS_orientation.pitch);Serial.print(" ");Serial.print(AHRS_orientation.yaw);Serial.print(" ");Serial.println(AHRS_orientation.roll);Serial.print(" ");Serial.print(Guidance_data.servoValueX);Serial.print(" ");Serial.println(Guidance_data.servoValueZ);
        //Serial.print(IMU_data.gx);Serial.print(" ");Serial.println(IMU_data.ax);
    }

    

}