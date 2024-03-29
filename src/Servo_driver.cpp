#include <Servo.h>
//#include <Arduino.h>
#include "Servo_driver.h"


Servo S1; 
Servo S2; 
Servo S3; 
Servo S4; 

void Servo_init(int S1Pin, int S2Pin, int S3Pin, int S4Pin){
    //Attach servos to the given pins
    S1.attach(S1Pin);
    S2.attach(S2Pin);
    S3.attach(S3Pin);
    S4.attach(S4Pin);
    

    //Test sequence
    S1.write(89);
    S2.write(85);   
    S3.write(89);
    S4.write(85);  

}


void Servo_execute(float servoValueS1, float servoValueS2, float servoValueS3, float servoValueS4){
    S1.write(servoValueS1-1);
    S2.write(servoValueS2-5);   
    S3.write(servoValueS3-1);
    S4.write(servoValueS4-5);  
}
