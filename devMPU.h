#include <Wire.h>
//#include <I2Cdev.h>
#include <Math.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#include <MPU6050.h>
//#include <avr/pgmspace.h>

//--------------REGULAR SETUP---------------//
#define LED_PIN 13 
#define MPU_RAW_ACCEL
#define accelAsComponents
//#define accelAsMag
#define MPU_RAW_GYRO

//--ABSOLUTE ANGLE (Complementary filter)--//
//#define ABSOLUTE_ANGLE
#define RADIANS_TO_DEGREES (180/3.14159)


//-----------INTERRUPT ROUTINE----------//
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

//-----------TAP DETECTION-------------//
//Tap time (in ms)
#define TAP_T 100
// Tap tresh bandwidth (in Hz)
#define TAP_THRESH_BW 100
//Tap thresh height (of signal)
#define TAP_THRESH_H 200

//Functions for tap detection
//#define PROGRAMMED_TAP
//#define MANUAL_TAP


//----------FALL DETECTION-------------//



//-------FUNCTION DEFINTIONS-----------//
uint8_t initializeMPU(int16_t *accelOffsetX, int16_t *accelOffsetY,int16_t *accelOffsetZ, int16_t *currAccelX,int16_t *currAccelY,int16_t *currAccelZ);
uint8_t mpuMonitor(int16_t *currAccelX,int16_t *currAccelY,int16_t *currAccelZ);
uint8_t mpuAcquire(int16_t *accelOffsetX, int16_t *accelOffsetY,int16_t *accelOffsetZ, int16_t *currAccelX, int16_t *currAccelY, int16_t *currAccelZ, int16_t *currYaw, int16_t *currPitch, int16_t *currRoll);
void dmpDataReady();