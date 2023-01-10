#pragma once

#include <Arduino.h>
#include <SD.h>
#include <String.h>


bool SD_init(int SdPin, bool DEBUG_OUTPUT);
void SD_writeData(String frame);
void SD_close();
