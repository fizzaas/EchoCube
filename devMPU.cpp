#include <Wire.h>
#include <I2Cdev.h>
#include <Math.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>
#include <mpu.h>
#include <avr/pgmspace.h>

//--------GLOBAL VARIABLES AND CONSTRUCTOR---------//
bool dmpReady = false;  // set true if DMP init was successful
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//uint8_t fifoBuffer[64]; // FIFO storage buffer

devMPU::devMPU(){
		
}
//------------------------------------------------//

void devMPU::initializeMPU(uint8_t *accelOffsetX, uint8_t *accelOffsetY,uint8_t *accelOffsetZ, uint8_t *currAccelX,uint8_t *currAccelY,uint8_t *currAccelZ){

uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

  // CONNECT DEVICE TO I2C BUS

// join I2C bus (I2Cdev library doesn't do this automatically)
  Wire::begin();

//  Serial.begin(38400);
//  while (!Serial);
//
//  // initialize device
//  Serial.println(F("Initializing I2C devices..."));
  MPU6050::initialize();


  // TEST MPU CONNECTION

//  // verify connection
//  Serial.println(F("Testing device connections..."));
//  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
//  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//  while (Serial.available() && Serial.read()); // empty buffer
//  while (!Serial.available());                 // wait for data
//  while (Serial.available() && Serial.read()); // empty buffer again


  // CONFIGURE MPU SETTINGS

  //Low pass filtering
  //MPU6050::setDLPFMode(1);

  //Set tap detection on XYZ axes
  /* dmp_set_tap_thresh(1,500);
  dmp_set_tap_thresh(2,500);
  dmp_set_tap_thresh(4,500); */

  
  
  // LOAD AND CONFIGURE THE DMP
  
//  Serial.println(F("Initializing DMP..."));
    devStatus=MPU6050::dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
//    Serial.println(F("Enabling DMP..."));
    MPU6050::setDMPEnabled(true);

    // enable Arduino interrupt detection
//    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 2)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    

	//mpuIntStatus = MPU6050::getIntStatus();
//    Serial.println("MPU int status:");
//    Serial.println(mpuIntStatus);

    // set our DMP Ready flag so function knows it's okay to use it
//    Serial.println(F("DMP ready!"));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = MPU6050::dmpGetFIFOPacketSize();
    
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
//    Serial.print(F("DMP Initialization failed (code "));
//    Serial.print(devStatus);
//    Serial.println(F(")"));
  }


  // RUN CALIBRATION

  // According to manual, user should place the cube on table for 10 seconds to allow for accelerometer to calibrate
  // Accelerometer calibration: apply offsets

  uint8_t tempOffsetX=0;
  uint8_t tempOffsetY=0;
  uint8_t tempOffsetZ=0;


  //Get offset as average over 10 seconds
  uint8_t count=0;

  *accelOffsetX=0;
  *accelOffsetY=0;
  *accelOffsetZ=0;
  unsigned long startTime=millis();
  while ((millis()-startTime)<10000){
    mpuMonitor(currAccelX,currAccelY,currAccelZ);
    tempOffsetX=(tempOffsetX+currAccelX/2048);
    tempOffsetY=(tempOffsetY+currAccelY/2048);
    tempOffsetZ=(tempOffsetZ+currAccelZ/2048);
    count++;
  }

  tempOffsetX=2048*tempOffsetX/count;
  tempOffsetY=2048*tempOffsetY/count;
  tempOffsetZ=2048*tempOffsetZ/count;

  *accelOffsetX=tempOffsetX;
  *accelOffsetY=tempOffsetY;
  *accelOffsetZ=tempOffsetz;

  pinMode(LED_PIN,output);
}



// ================================================================
// ===       DATA ACQUISITION AND PROCESSED OUTPUT              ===
// ================================================================

// PRIVATE METHOD TO MONITOR FOR DATA RECEIVED
// Blink LED on Arduino to indicate problems instead of serial printing during actual operation, due to UART already being in use

bool devMPU::mpuMonitor(uint8_t *currAccelX,uint8_t *currAccelY,uint8_t *currAccelZ){

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t fifoCount;     // count of all bytes currently in FIFO

	
  //PROGRAMMING FAILURE CHECK
  bool blinkState=0;
  
  if (!dmpReady) return false;
  // HOLD LED ON TO INDICATE PROBLEM
    blinkState = 1;
    digitalWrite(LED_PIN, blinkState);

  //NO-INTERRUPT CHECK
  // wait for MPU interrupt or extra packet(s) available
  if(!mpuInterrupt && (fifoCount < packetSize)){
    return false;
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = MPU6050::getIntStatus();

  // get current FIFO count
  fifoCount = MPU6050::getFIFOCount();

  // FIFO OVERFLOW CHECK
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // mpu FIFO OFLOW flag is raised or fifoCount has max of 1024 (max # of bytes in buffer)
    // BLINK LED 5x TO INDICATE OVERFLOW
    for(uint8_t i=0; i<6; i++){
      blinkState = ~blinkstate;
      digitalWrite(LED_PIN, blinkState);
      delay(100);
    }
	// reset so we can continue cleanly
    MPU6050::resetFIFO();
    return false;
  }
  
  
  // GOT MPU DATA READY INTERRUPT WITH SUFFICIENT SIZE!
  else if (mpuIntStatus & 0x02) {

    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = MPU6050::getFIFOCount();

    // read a packet from FIFO
    //MPU6050::getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
	
	//mpuMONITOR gets values of acceleration, too...
	//to provide for offset calculation in calibration (redundancy - to improve)
	MPU6050::getAcceleration(currAccelX,currAccelY,currAccelZ);

    return true;
  }

  return false;
}





// PUBLIC METHOD TO PROVIDE DATA ACQUIRED FROM MPU TO DEPENDENT SYSTEMS
void devMPU::mpuAcquire(uint8_t *accelOffsetX, uint8_t *accelOffsetY,uint8_t *accelOffsetZ, uint8_t *currAccelX, uint8_t *currAccelY, uint8_t *currAccelZ, uint8_t *currYaw, uint8_t *currPitch, uint8_t *currRoll){

int16_t rot[3]; //Equivalent to uint8_t yaw; uint8_t pitch; uint8_t roll;
int16_t accel[3];//Equivalent to uint8_t accelX; uint8_t accelY; uint8_t accelZ;

//DEFAULT SEND 0's
for(uint8_t j=0; j<4;j++){
  accel[j]=0;
}

for(uint8_t k=0; k<4;k++){
  rot[k]=0;
}

//IF PROBLEM, SEND PREVIOUS VALUES
    if(!mpuMonitor(currAccelX,currAccelY,currAccelZ)){
       accel[0]=*currAccelX;
       accel[1]=*currAccelY;
       accel[2]=*currAccelZ;

       rot[0]=*currYaw;
       rot[1]=*currPitch;
       rot[2]=*currRoll;
    }

//IF GOT VALUES, SUPPLY NEW VALUES
    else if(mpuMonitor(currAccelX,currAccelY,currAccelZ)){

	// display raw acceleration values
    // Chosen getAcceleration method because only one showing approximately independent dimensions
    // ALL others having problems; linearAccelInWorld in particular only giving zeroes, and linearAccel giving all ~same
    #ifdef MPU_RAW_ACCEL
    MPU6050::getAcceleration(accel,accel+1,accel+2);
    accel[0]=(accel[0] - *accelOffsetX) // Components with offset applied
    accel[1]=(accel[1] - *accelOffsetY) // No scaling factor 2048, to maximize resolution in integer rep.
    accel[2]=(accel[2] - *accelOffsetZ)
    #endif

	// TO DO: WILL DETERMINE AXIS OF MAX DIFFERENCE BETWEEN accel AND currAccel VALUES...
	// AND DIVIDE OTHER AXES' VALUES BY DERIVATIVE (USING TIMER COUNT)
	
	// -OR- TAKE TIME AVERAGE OF BUFFERED VALUES AND DIVIDE OTHERS BY AXIS OF MAX RATE OF CHANGE
	
	// -OR- MAX/MIN HOLD VALUES ON AXIS AND FIND RATE OF CHANGE OF PEAKS. DIVIDE OTHER AXES BY MAX RATE OF CHANGE
	
	
    // display raw gyroscope values
    //Took raw values because all others not independent (and dmpGetGyro in particular not sensitive)
    #ifdef MPU_RAW_GYRO 
    MPU6050::getRotation(rot,rot+1,rot+2);
    /* rot[0]=(rot[0]/16.4); // No scaling of rotation, so as maximize resolution with integer
    rot[1]=(rot[1]/16.4);
    rot[2]=(rot[2]/16.4); */

      //Return absolute angle values, if desired
      /* #ifdef ABSOLUTE_ANGLE
      //Apply complementary filter to get
      Complementary_Filter(accel[0],accel[1],accel[2],rot[0],rot[1],rot[2]);
      #endif */
    
    #endif

    }


//Print out and update "previous" values to current ones
#ifdef MPU_RAW_ACCEL
	#ifdef accelAsComponents
		Serial.print(accel[0] + ", " + accel[1] + ", " + accel[2]));
	#endif
	#ifdef accelAsMag
        Serial.print(sqrt((abs(accel[0]))^2+(abs(accel[1]))^2+(abs(accel[2]))^2));
	#endif
*currAccelX=accel[0];
*currAccelY=accel[1];
*currAccelZ=accel[2];
#endif

#ifdef MPU_RAW_GYRO
Serial.print(rot[0] + ", " + rot[1] + ", " + rot[2]);
*currYaw=rot[0];
*currPitch=rot[1];
*currRoll=rot[2];
#endif


/* #ifdef GET_TAPS
    if(&(devAddr+TAPXYZ)){ //If tap received register goes high, send 1 tap indication
      Serial.print(", 1");
    }
	#ifdef MANUAL_TAP
    else if(gotTaps(rot[0],rot[1],rot[2],sFIFO_Y,sFIFO_P,sFIFO_R)){ //Same from gotTap() function, if returns true
      Serial.print(", 1");
    }
	#endif
#endif */

}



// ================================================================
// ===               INTERRUPT SERVICE ROUTINE                ===
// ================================================================

void devMPU::dmpDataReady() {
  mpuInterrupt = true;
}


// ================================================================
// ===               PERSONAL DEV FUNCTIONS                     ===
// ================================================================

/* bool devMPU::gotTaps(uint8_t yaw,uint8_t pitch,uint8_t roll,SimpleFIFO<int,100> sFIFO_Y,SimpleFIFO<int,100> sFIFO_P,SimpleFIFO<int,100> sFIFO_R){

    uint8_t yBW=0;
    uint8_t pBW=0;
    uint8_t rBW=0;
    
    sFIFO_Y.enqueue(yaw);
    sFIFO_P.enqueue(pitch);
    sFIFO_R.enqueue(roll);

    //Get BW of 100 taps
    yBW=DFT_BW(sFIFO_Y,sFIFO_P,sFIFO_R); //Still need to implement FFT_BW
    pBW=DFT_BW(sFIFO_P); //Still need to implement FFT_BW
    rBW=DFT_BW(sFIFO_R); //Still need to implement FFT_BW

    //Max of taps
    uint8_t max[3];
    for(int i=0; i<101;i++){
      if(sFIFO_Y.peek()>max[0]){
        max[0]=sFIFO_Y.peek();
      }
    }
    
	//If BW of sample reaches cutoff and peak is sufficiently high, got pulse
    if((yBW>TAP_THRESH_BW && max[0]>TAP_THRESH_H)||(pBW>TAP_THRESH_BW && max[1]>TAP_THRESH_H)||(rBW>TAP_THRESH_BW && max[2]>TAP_THRESH_H)){ 
      return true;
    }
	
	return false;
}

// ================================================================
// ===               LIBRARY FUNCTIONS                          ===
// ================================================================

void devMPU::Complementary_Filter(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y,float gyro_z){
        float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_z = 0;

        // Compute the (filtered) gyro angles
        float dt =(t_now - get_last_time())/1000.0;
        float gyro_angle_x = gyro_x*dt + get_last_x_angle();
        float gyro_angle_y = gyro_y*dt + get_last_y_angle();
        float gyro_angle_z = gyro_z*dt + get_last_z_angle();
        
        // Compute the drifting gyro angles
        float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
        float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
        float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();     
        
        // Apply the complementary filter to figure out the change in angle - choice of alpha is
        // estimated now.  Alpha depends on the sampling rate...
        const float alpha = 0.96;
        float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
        float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
        float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
        
        // Update the saved data with the latest values
        set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);       
    } */
