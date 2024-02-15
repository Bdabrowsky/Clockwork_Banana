/*
#
#   Board config
#
*/

#define Buzzer 4
#define P1_EN 10
#define P2_EN 12
#define CS_SD 2
#define CS_LSM 5
#define CS_FLASH 2137


/*
#
#   Code config
#
*/

//Tuning parameters, DO NOT TOUCH ANYTHING ELSE!
bool            DEBUG_OUTPUT=                           true;                       //Choose if computer should work in debug mode, printing logs on serial


//Gyroscope settings
const int             gyroCalibrationSampleCount=       300;                        //Amount of samples taken for gyroscope calibration. 1000 is about 5-10 seconds of calibration
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
const float           mainAltitude=                     100000;
const int             apogeeCnt=                        100;

//Refresh rate settings
int                   dataLoggingFREQ=                  2;                          //SD data logging frequency during ascent in HZ, on the ground 1/4 of that
unsigned long         mainLoopFREQ=                     100;                         //Main logic loop frequency in HZ
int                   buzzerFREQ=                       1;                          //frequency of buzzing during ground operation, 2 times thst after liftoff, 4 times that on apogee, 32 on error
int                   debugFREQ=                        5; 


