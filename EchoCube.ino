/* Main program for the EchoCube third year project. */

/* Import the required libraries. */
#include "devMPU.h"

/* Global variables (boo hiss!) */
devMPU mpu;
uint8_t offset_data[6] = {0,0,0,0,0,0};
int16_t mpu_data[6] = {0,0,0,0,0,0};

void setup() {
  /* Initialize the ESP and verify successful connection. */

  /* Initialize the MPU and verify successful initialization. */
  bool mpu_init = false; uint8_t mpu_arb = 0;
  while (!mpu_init && mpu_arb < 10)
  {
    mpu_init = mpu.initializeMPU(&(offset_data[0]), &(offset_data[1]), &(offset_data[2]), &(offset_data[3]), &(offset_data[4]), &(offset_data[5]));
    mpu_arb++;
  }
  /* Make sure the link to the server is active. */
  while (!esp.link()) { delay(5000); }
}

void loop() {
  /* Read data */
  mpu.mpuAcquire(&(offset_data[0]), &(offset_data[1]), &(offset_data[2]), &(mpu_data[0]), &(mpu_data[1]), &(mpu_data[2]), &(mpu_data[3]), &(mpu_data[4]), &(mpu_data[5]));
  /* Process data */
  String send_data = "";
  for (int i = 0; i < 6; i++)
  {
    String append = String(mpu_data[i]);
    send_data.concat(append);
  }
  String send_len = String(send_data.length());
  /* Send data */
  no_link_detected = esp.sendData(send_len, send_data);
  // If we experienced an unsuccessful send data
  if (no_link_detected)
  {
    while (!esp.link()){}
  }
  
}
