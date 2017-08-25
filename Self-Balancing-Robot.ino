#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <Wire.h>

void interrupt(void);

volatile unsigned char interruptAvailable;

struct int_param_s params = {.pin=2, .cb=interrupt};

void interrupt()
{
  interruptAvailable = 1;
}

void mpu_calculate_biases()
{
  long gyrocal[3], accelcal[3];
  
    mpu_run_self_test(gyrocal, accelcal);
  
    accelcal[0] = (long)((accelcal[0]/65536.0) * 2048);
    accelcal[1] = (long)((accelcal[1]/65536.0) * 2048);
    accelcal[2] = (long)((accelcal[2]/65536.0) * 2048);
  
    gyrocal[0] = (long)((gyrocal[0]/65536.0) * 32.8);
    gyrocal[1] = (long)((gyrocal[1]/65536.0) * 32.8);
    gyrocal[2] = (long)((gyrocal[2]/65536.0) * 32.8);
    
    mpu_set_accel_bias_6050_reg(accelcal);
    mpu_set_gyro_bias_reg(gyrocal);
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);

  mpu_init(&params);
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(1000);

  mpu_calculate_biases();

  dmp_load_motion_driver_firmware();
  
  dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_TAP | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO);

  dmp_set_fifo_rate(100);
    
  mpu_set_dmp_state(1);

}

void loop() {
  if(interruptAvailable)
  {
    interruptAvailable = 0;

  }
}
