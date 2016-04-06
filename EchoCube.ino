/* Main program for the EchoCube third year project. */

/* Import the required libraries. */
#include "devMPU.h"

/* Global variables (boo hiss!) */
devMPU mpu;
uint8_t offset_data[6] = {0,0,0,0,0,0};

void setup() {
  /* Initialize the ESP and verify successful connection. */

  /* Initialize the MPU and verify successful initialization. */
  bool mpu_init = false; uint8_t mpu_arb = 0;
  while (!mpu_init && mpu_arb < 10)
  {
    mpu_init = mpu.initializeMPU(&(offset_data[0]), &(offset_data[1]), &(offset_data[2]), &(offset_data[3]), &(offset_data[4]), &(offset_data[5]));
    mpu_arb++;
  }
}

void loop() {
  

}
