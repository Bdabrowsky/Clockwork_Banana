#include "SD_driver.h"
#include <SD.h>

File meas;

bool SD_init(int SdPin, bool DEBUG_OUTPUT){
    if(!SD.begin(SdPin)){
        if(DEBUG_OUTPUT){
           Serial.println("Could not find SD card, check if present!");
        }
        return false;
    }

    meas = SD.open("LOG.csv", FILE_WRITE);

    if(meas.size() > 20){
        if(DEBUG_OUTPUT){
            Serial.println("SD card not empty!");
        }   
        return false;
    }

    meas.println("Timestamp,State,ACCx,ACCy,ACCz,GYROx,GYROy,GYROz,Altitude,MaxAltitude,Pitch,Yaw,Roll");

    if(DEBUG_OUTPUT){
        Serial.println("Found SD card!");
    }
        
    return true;
}

void SD_writeData(char *frame){
    meas.println(frame);
}

void SD_close(){
    meas.close();
}