#pragma once

#include <Arduino.h>
#include <SD.h>



bool SD_init(int SdPin, bool DEBUG_OUTPUT);
void SD_writeData(char *frame);
void SD_close();
