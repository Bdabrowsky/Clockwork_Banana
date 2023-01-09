#pragma once

//#include <Arduino.h>
#include <Servo.h>

void Servo_init(int S1Pin, int S2Pin, int S3Pin, int S4Pin);
void Servo_execute(float servoValueS1, float servoValueS2, float servoValueS3, float servoValueS4);



