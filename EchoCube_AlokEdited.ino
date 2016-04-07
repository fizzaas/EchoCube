/* Main program for the EchoCube third year project. */

/* Import the required libraries. */
#include "ESP8266.h"
#include <SoftwareSerial.h>
#include "Math.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

/* Configurable deginitions */
#define SSID        "Nagui's Network"
#define PASSWORD    "19520605"
#define MPU_RAW_ACCEL
#define accelAsComponents
#define MPU_RAW_GYRO
#define RADIANS_TO_DEGREES (180/3.14159)
//-----------INTERRUPT ROUTINE----------//
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//-----------TAP DETECTION-------------//
//Knock threshold (1LSB/2mg)
#define KNOCK_THRESH 100
//Knock time threshold (1LSB/1ms)
#define KNOCK_DUR 10
//Fall threshold (1LSB/2mg)
#define FALL_THRESH 8000
//Fall time threshold (1LSB/1ms)
#define FALL_DUR 30

/* Global variables (boo hiss!) */
MPU6050 mpu;
int16_t offset_data[6] = {0,0,0,0,0,0};
int16_t mpu_data[6] = {0,0,0,0,0,0};
bool *got_knock;
bool *now_falling;
SoftwareSerial mySerial(10, 11); // RX, TX from microcontroller perspective
ESP8266 wifi(mySerial); //creating ESP object
//--------GLOBAL VARIABLES---------//
bool dmpReady = false;  // set true if DMP init was successful
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//uint8_t fifoBuffer[64]; // FIFO storage buffer

void setup() {
  Serial.begin(4800);
  pinMode(13,OUTPUT);
  /* Initialize the ESP and verify successful connection. */
  bool esp_init = false; uint8_t esp_arb = 0;
  while (!esp_init && esp_arb < 5)
  {
    esp_init = initESP();
    Serial.println(esp_init);
    esp_arb++;
  }
  /* Initialize the MPU and verify successful initialization. */
  uint8_t mpu_init = 1; uint8_t mpu_arb = 0;
  while (mpu_init != 0 && mpu_arb < 10)
  {
    mpu_init = initializeMPU(&(offset_data[0]), &(offset_data[1]), &(offset_data[2]), &(offset_data[3]), &(offset_data[4]), &(offset_data[5]),got_knock,now_falling);
    Serial.println(mpu_init);
    mpu_arb++;
  }
  /* Make sure the link to the server is active. */
  while (!linkPresent()) { delay(5000); Serial.println("stuck help!"); }
  //digitalWrite(13,HIGH);
}

void loop() {
  /* Read data */
  mpuAcquire(&(offset_data[0]), &(offset_data[1]), &(offset_data[2]), &(mpu_data[0]), &(mpu_data[1]), &(mpu_data[2]), &(mpu_data[3]), &(mpu_data[4]), &(mpu_data[5]),got_knock,now_falling);
  Serial.println(mpu_data[1]);
  /* Process data */
  String send_data = "";
  for (int i = 0; i < 6; i++)
  {
    String append = String(mpu_data[i]);
    append += " ";
    send_data.concat(append);
  }
  //Append knock=1 or 0
  String appendKnock = String( (int16_t)*got_knock );
  appendKnock += " ";
  //Append falling=1 or 0
  send_data.concat(appendKnock);
  String appendFalling = String( (int16_t)*now_falling );
  appendKnock += " ";
  send_data.concat(appendKnock);
  
  String send_len = String(send_data.length());
  /* Send data */
  boolean link_detected = sendData(send_len, send_data);
  Serial.println(link_detected);
  // If we experienced an unsuccessful send data
  if (!link_detected)
  {
    delay(500);
    //while (!linkPresent()) { delay(5000); }
  }
  delay(200);
}

boolean initESP(void)
{
    mySerial.begin(4800);
    delay(1000);
    
    if (wifi.setOprToStation()) {
        //do nothing
    } else {
        return false;
    }
 
    if (wifi.joinAP(SSID, PASSWORD)) {
       //do nothing  
    } else {
        return false;
    }
    
    if (wifi.enableMUX()) {
       //do nothing
    } else {
        return false;
    }
    
    if (wifi.startTCPServer(80)) {
        //do nothing
    } else {
        return false;
    }
    return true; //everything ok if we reach this point
}//end function
 
    
boolean linkPresent(void){
  if(mySerial.find("Link")){
    return true;
  }else{
    return false;
  }//end if
}//end function

boolean sendData(String size, String data){
    //sending size and link number
    String cmd="AT+CIPSEND=0,"+size+"\r\n";
    mySerial.print(cmd); 

    //sending data
    if(mySerial.find(">")){
    mySerial.println(data);
    }
  
   if(mySerial.find("SEND OK")){
    return true;
   }else{
    return false;
    }//end if
}//end function

uint8_t initializeMPU(int16_t *accelOffsetX, int16_t *accelOffsetY,int16_t *accelOffsetZ, int16_t *currAccelX,int16_t *currAccelY,int16_t *currAccelZ,bool *got_knock, bool *now_falling){

/*MUST DECIDE WHETHER TO USE*/
/*ERROR CODES:
1: no connection of MPU in hardware
2: no DMP initialization
*/
uint8_t initErrorCode=0;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

  // CONNECT DEVICE TO I2C BUS

// join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

//  Serial.begin(38400);
//  while (!Serial);
//
//  // initialize device
//  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();


  // TEST MPU CONNECTION

//  // verify connection
  if( !(mpu.testConnection()) ){ /*Added MPU connection check*/
    initErrorCode=1;
    return initErrorCode;
  }
//  Serial.println(F("Testing device connections..."));
//  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
//  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//  while (Serial.available() && Serial.read()); // empty buffer
//  while (!Serial.available());                 // wait for data
//  while (Serial.available() && Serial.read()); // empty buffer again


  // CONFIGURE MPU SETTINGS

    // zero motion threshold (for knock detection)
    mpu.setZeroMotionDetectionThreshold(KNOCK_THRESH);//1LSB/ 2mg threshold?
    mpu.setZeroMotionDetectionDuration(KNOCK_DUR);//1LSB / 1ms threshold time   

    //falling detection threshold
    mpu.setFreefallDetectionThreshold(FALL_THRESH);//1LSB / 2mg threshold
    mpu.setFreefallDetectionDuration(FALL_DUR);//1LSB / 1ms threshold time

  
  
  // LOAD AND CONFIGURE THE DMP
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//  Serial.println(F("Initializing DMP..."));
    devStatus=mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
//    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
//    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 2)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    

  //mpuIntStatus = mpu.getIntStatus();
//    Serial.println("MPU int status:");
//    Serial.println(mpuIntStatus);

    // set our DMP Ready flag so function knows it's okay to use it
//    Serial.println(F("DMP ready!"));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  } else {
    /*Failed to intialize dmp*/
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
//    Serial.print(F("DMP Initialization failed (code "));
//    Serial.print(devStatus);
//    Serial.println(F(")"));
    initErrorCode=2;
    return initErrorCode; 

  }



  // RUN CALIBRATION

  // According to manual, user should place the cube on table for 1 second to allow for accelerometer to calibrate
  // Accelerometer calibration: apply offsets

  int16_t tempOffsetX=0;  /*CHANGED TO INT16*/
  int16_t tempOffsetY=0;
  int16_t tempOffsetZ=0;


  //Get offset as average over 10 seconds
  uint8_t count=0;

  *accelOffsetX=0;
  *accelOffsetY=0;
  *accelOffsetZ=0;
  unsigned long startTime=millis();
  while ((millis()-startTime)<1000){
    Serial.println("Error code mpu:");
    Serial.println(mpuMonitor(currAccelX,currAccelY,currAccelZ,got_knock,now_falling));
    tempOffsetX=(tempOffsetX+*(currAccelX)/2048);
    tempOffsetY=(tempOffsetY+*(currAccelY)/2048);
    tempOffsetZ=(tempOffsetZ+*(currAccelZ)/2048);
    count++;
  }

  tempOffsetX=2048*tempOffsetX/count;
  tempOffsetY=2048*tempOffsetY/count;
  tempOffsetZ=2048*tempOffsetZ/count;

  *accelOffsetX=tempOffsetX;
  *accelOffsetY=tempOffsetY;
  *accelOffsetZ=tempOffsetZ;

  //pinMode(LED_PIN,output); /*No more LEDs for failure checks*/

  return initErrorCode;
}



// ================================================================
// ===       DATA ACQUISITION AND PROCESSED OUTPUT              ===
// ================================================================

// PRIVATE METHOD TO MONITOR FOR DATA RECEIVED
// Blink LED on Arduino to indicate problems instead of serial printing during actual operation, due to UART already being in use

uint8_t mpuMonitor(int16_t *currAccelX,int16_t *currAccelY,int16_t *currAccelZ, bool *got_knock, bool *now_falling){

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t fifoCount;     // count of all bytes currently in FIFO
/*MUST DECIDE WHETHER TO USE*/
/*ERROR CODES:
1: DMP not ready
2: No interrupt received
3: FIFO OFLOW
4: Other (unknown)
*/
uint8_t monitorErrorCode=0;

*got_knock=false;
*now_falling=false;
  
  //PROGRAMMING FAILURE CHECK
  if (!dmpReady){
    monitorErrorCode=1;
    return monitorErrorCode;
  }
  
  //NO-INTERRUPT CHECK
  // If fails, must wait for MPU interrupt or extra packet(s) to become available
  // Also catches if interrupt line disconnected, or other hardware issues (e.g. power loss)
  if(!mpuInterrupt && (fifoCount < packetSize)){
    monitorErrorCode=2;
    return monitorErrorCode;
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // FIFO OVERFLOW CHECK
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // mpu FIFO OFLOW flag is raised or fifoCount has max of 1024 (max # of bytes in buffer)
   //monitorErrorCode=3; 
  // reset so we can continue cleanly
    mpu.resetFIFO();
    return monitorErrorCode;
  }


  // GOT MPU DATA READY INTERRUPT WITH SUFFICIENT SIZE!
  
  //CHECK knock detection status
  if(mpuIntStatus & 0x80){
    *got_knock=true;
  }
  else{
    *got_knock=false;
  }
  //CHECK free fall status
  if(mpuIntStatus & 0x20 && !(mpuIntStatus & 0x80)){
    *now_falling=true;
  }
  else{
    *now_falling=false;
  }
  
  if (mpuIntStatus & 0x02) {

    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    //mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
  
    //mpuMONITOR gets values of acceleration, too...
    //to provide for offset calculation in calibration (redundancy - to improve)
    mpu.getAcceleration(currAccelX,currAccelY,currAccelZ);

    return monitorErrorCode;
  }

  //Unknown error
  monitorErrorCode=4;

  return monitorErrorCode;
}





// PUBLIC METHOD TO PROVIDE DATA ACQUIRED FROM MPU TO DEPENDENT SYSTEMS
uint8_t mpuAcquire(int16_t *accelOffsetX, int16_t *accelOffsetY,int16_t *accelOffsetZ, int16_t *currAccelX, int16_t *currAccelY, int16_t *currAccelZ, int16_t *currYaw, int16_t *currPitch, int16_t *currRoll,bool *got_knock, bool *now_falling){ /*Changed to int16*/

int16_t rot[3]; //Equivalent to uint8_t yaw; uint8_t pitch; uint8_t roll;
int16_t accel[3];//Equivalent to uint8_t accelX; uint8_t accelY; uint8_t accelZ;
//Error flag
uint8_t errorCode;

//DEFAULT SEND 0's
for(uint8_t j=0; j<4;j++){
  accel[j]=0;
}

for(uint8_t k=0; k<4;k++){
  rot[k]=0;
}

//Monitor MPU
errorCode=mpuMonitor(currAccelX,currAccelY,currAccelZ,got_knock,now_falling);

Serial.println("Error code:");
Serial.println(errorCode);

//IF PROBLEM, SEND PREVIOUS VALUES
    if(errorCode!=0){
       accel[0]=*currAccelX;
       accel[1]=*currAccelY;
       accel[2]=*currAccelZ;

       rot[0]=*currYaw;
       rot[1]=*currPitch;
       rot[2]=*currRoll;
    }

//IF GOT VALUES, SUPPLY NEW VALUES
    else if(errorCode==0){

  // display raw acceleration values
    // Chosen getAcceleration method because only one showing approximately independent dimensions
    // ALL others having problems; linearAccelInWorld in particular only giving zeroes, and linearAccel giving all ~same
    #ifdef MPU_RAW_ACCEL
    mpu.getAcceleration(accel,accel+1,accel+2);
    accel[0]=(accel[0] - *accelOffsetX); // Components with offset applied
    accel[1]=(accel[1] - *accelOffsetY); // No scaling factor 2048, to maximize resolution in integer rep.
    accel[2]=(accel[2] - *accelOffsetZ);
    #endif

  // TO DO: WILL DETERMINE AXIS OF MAX DIFFERENCE BETWEEN accel AND currAccel VALUES...
  // AND DIVIDE OTHER AXES' VALUES BY DERIVATIVE (USING TIMER COUNT)
  
  // -OR- TAKE TIME AVERAGE OF BUFFERED VALUES AND DIVIDE OTHERS BY AXIS OF MAX RATE OF CHANGE
  
  // -OR- MAX/MIN HOLD VALUES ON AXIS AND FIND RATE OF CHANGE OF PEAKS. DIVIDE OTHER AXES BY MAX RATE OF CHANGE
  
  
    // display raw gyroscope values
    //Took raw values because all others not independent (and dmpGetGyro in particular not sensitive)
    #ifdef MPU_RAW_GYRO 
    mpu.getRotation(rot,rot+1,rot+2);
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

/*  FOR DEBUG, SERIAL PRINT
  #ifdef accelAsComponents
    Serial.print(accel[0] + ", " + accel[1] + ", " + accel[2]));
  #endif
  #ifdef accelAsMag
        Serial.print(sqrt((abs(accel[0]))^2+(abs(accel[1]))^2+(abs(accel[2]))^2));
  #endif
*/

  *currAccelX=accel[0];
  *currAccelY=accel[1];
  *currAccelZ=accel[2];
#endif


#ifdef MPU_RAW_GYRO
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


//Return error code, to indicate type of problem if any
return errorCode;

}



// ================================================================
// ===               INTERRUPT SERVICE ROUTINE                ===
// ================================================================

void dmpDataReady() {
  mpuInterrupt = true;
}
        
