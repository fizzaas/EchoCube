#include <Wire.h>
#include <I2Cdev.h>
#include <Math.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>

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
class devMPU:MPU6050 {
    public:
void devMPU::initializeMPU(uint8_t *accelOffsetX, uint8_t *accelOffsetY,uint8_t *accelOffsetZ, uint8_t *currAccelX,uint8_t *currAccelY,uint8_t *currAccelZ);
bool devMPU::mpuMonitor(uint8_t *currAccelX,uint8_t *currAccelY,uint8_t *currAccelZ);
void devMPU::mpuAcquire(uint8_t *accelOffsetX, uint8_t *accelOffsetY,uint8_t *accelOffsetZ, uint8_t *currAccelX, uint8_t *currAccelY, uint8_t *currAccelZ, uint8_t *currYaw, uint8_t *currPitch, uint8_t *currRoll);
void dmpDataReady();
}