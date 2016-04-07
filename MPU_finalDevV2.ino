// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
//#include <I2Cdev.h>

#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h> // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;


//#define MPU_RAW_GYRO
#define MPU_RAW_ACCEL



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int16_t rot[3];
int16_t accel[3];

int16_t maxAccel[3];
int16_t minAccel[3];
uint8_t count[3];
bool prevMotionless[3];


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(38400);
  while (!Serial);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//  while (Serial.available() && Serial.read()); // empty buffer
//  while (!Serial.available());                 // wait for data
//  while (Serial.available() && Serial.read()); // empty buffer again




  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  //devStatus = 0;

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println("MPU int status:");
    Serial.println(mpuIntStatus);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    mpu.resetFIFO();



    // zero motion threshold (for knock detection)
    mpu.setZeroMotionDetectionThreshold(1000);//25 because 50mg threshold?
    mpu.setZeroMotionDetectionDuration(10);//500 because 500ms threshold time?


    //zero motion detection threshold (for knock detection)
    //mpu.setDHPFMode(500);
    mpu.setZeroMotionDetectionThreshold(1000);//25 because 50mg threshold?
    mpu.setZeroMotionDetectionDuration(10);//500 because 500ms threshold time?

    //falling detection threshold
    mpu.setFreefallDetectionThreshold(8000);//25 because 50mg threshold?
    mpu.setFreefallDetectionDuration(30);//500 because 500ms threshold time?    

    //initialize for offset detection
    for(int j=0; j<3; j++){
      count[j]=1;
      prevMotionless[j]=1;
      maxAccel[j]=-32767;
      minAccel[j]=32767;
    }
    
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }



  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;


  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {// Do anything else here
    Serial.print("Int:\t");
    Serial.println(mpuInterrupt);
  }

  // reset interrupt flag and status, and then get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
//  //CHECK knock detection status
//  if(mpuIntStatus & 0x80){
//    Serial.println("Falling");
//    //delay(500);
//  }
//  //CHECK free fall status
//  if(mpuIntStatus & 0x20 && !(mpuIntStatus & 0x80)){
//    Serial.println("Knock");
//    //delay(500);
//  }

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // get current FIFO count
  fifoCount = mpu.getFIFOCount();
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.print(F("\t\tFIFO overflow!"));
  }

  
  // OTHERWISE, IF GOT DMP data ready interrupt (this should happen frequently)...
  else if ((mpuIntStatus & 0x02) || (mpuIntStatus & 0x01)) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize){
      fifoCount = mpu.getFIFOCount();
    }

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


     
    #ifdef MPU_RAW_GYRO
        //Serial.print("\n\n\n");
        //Serial.print("MPU data: ");
        
        //Chosen getRotation because all others not independent (and dmpGetGyro in particular not sensitive)
        //ALSO, NO NEED FOR DMP
        mpu.getRotation(rot,rot+1,rot+2);
        Serial.print(rot[0]);
        Serial.print(", ");
        Serial.print(rot[1]);
        Serial.print(", ");
        Serial.print(rot[2]);
        Serial.print(", ");
    #endif

    #ifdef MPU_RAW_ACCEL
        
        //#define accelAsComponents
        //#define accelAsMag

        // Chosen getAcceleration because only one showing approximately independent dimensions
        // ALSO, NO NEED FOR DMP
        // ALL others having problems; linearAccelInWorld in particular only giving zeroes, and linearAccel giving all ~same
        // Alternative: USE dmpGetAccel()

        mpu.getAcceleration(accel,accel+1,accel+2);


        /*ALGORITHM TO DETERMINE, REMOVE OFFSET*/
        //IF RANGE OF FLUCTUTATIONS (for a certain count) IS LESS THAN A CERTAIN VALUE...
        //SET ACCEL TO ZERO (offset region only)
        for(int i=0; i<3; i++){
          if(accel[i]>maxAccel[i]){
            maxAccel[i]=accel[i];
          }
          if(accel[i]<minAccel[i]){
            minAccel[i]=accel[i];
          }
          
          //Serial.print(abs(maxAccel[i]-minAccel[i]));
          Serial.print(maxAccel[i]);
          Serial.print(", ");
          //Serial.print(abs((maxAccel[i]-minAccel[i])/count[i]));
          Serial.print(minAccel[i]);
          Serial.print(", ");
          Serial.print(count[i]);
          Serial.print(",i=");
          Serial.print(i);
          if(i==2){
            Serial.println("\n");
          }
          
          if( abs((maxAccel[i]-minAccel[i])/count[i]) <= 1){
            accel[i]=0;
            prevMotionless[i]=1;
            maxAccel[i]=-32767;
            minAccel[i]=32767;
          }else{
            //Serial.print("D:");
            if(prevMotionless[i]){
              count[i]=1;
              prevMotionless[i]=0;
            }else{
              count[i]=count[i]+1;
            }
          }
        }


        
        
        #ifdef accelAsComponents
        
        Serial.print(accel[0] - 11);
        Serial.print(", ");
        Serial.print(accel[1] +8);
        Serial.print(", ");
        Serial.print(accel[2] - 3);
        Serial.println("\n");
        #endif
    
        #ifdef accelAsMag
        #define accelX (accel[0])
        #define accelY (accel[1])
        #define accelZ (accel[2])
        //Serial.println(accelX);
        Serial.print(sqrt((abs(accelX))^2+(abs(accelY))^2+(abs(accelZ))^2));
        #endif
    
    #endif

    //Serial.println("\n");

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }

  //If OTHER interrupt, display code

  else if( !((mpuIntStatus & 0x01) || (mpuIntStatus & 0x10) || (mpuIntStatus & 0x20) || (mpuIntStatus & 0x80)) ){
    Serial.print("\nOther interrupt:");
    Serial.print(mpuIntStatus, BIN);
  }

  //Interrupt no longer needed
  mpuIntStatus = 0;
}


