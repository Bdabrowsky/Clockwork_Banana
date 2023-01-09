#pragma once

typedef struct{
    int groundPressure;
    bool DEBUG_OUTPUT;

} BMP_config_t;

typedef struct{
    int pressure;
    float temperature;
    float altitude;
    float prevAltitude;
    float maxAltitude;
} BMP_data_t;



BMP_data_t getAltitude();
bool initBaro(bool DEBUG_OUTPUT);

     







 
