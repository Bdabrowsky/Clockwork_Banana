#include "Baro.h"
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

void barometerCalibration();

BMP_config_t BMP_config_d;
BMP_data_t BMP_data_d;

BMP_data_t getAltitude()
{
    float alt;

    alt = bmp.readAltitude(BMP_config_d.groundPressure/100.0f);
    
    if(isnan(alt))
    {
        return BMP_data_d;
    }

    float a=0.89f;

    BMP_data_d.altitude = BMP_data_d.prevAltitude + a * (alt - BMP_data_d.prevAltitude);

    BMP_data_d.maxAltitude = max(BMP_data_d.maxAltitude, BMP_data_d.altitude);

    BMP_data_d.prevAltitude = BMP_data_d.altitude;

    return BMP_data_d;

} 


void barometerCalibration()
{
    BMP_config_d.groundPressure = bmp.readPressure();
}

bool initBaro(bool DEBUG_OUTPUT)
{
    if(!bmp.begin(0x76)){
        if(DEBUG_OUTPUT)
            Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        return false;
    }
    if(DEBUG_OUTPUT){
        Serial.println("Found BMP sensor!");
    }

    barometerCalibration();

    /* Default settings from the datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    return true;

}
