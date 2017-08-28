#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <Wire.h>

void interrupt(void);

volatile unsigned char interruptAvailable;

struct int_param_s params = {.pin=2, .cb=interrupt};

short gyro[3];
short accel[3];
long quat[4];
unsigned long timestamp;
unsigned short sensors;
unsigned char more;


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

void quat_to_euler(float* quat_float, float* euler){
  double sinr = +2.0 * (quat_float[0] * quat_float[1] + quat_float[2] * quat_float[3]);
  double cosr = +1.0 - 2.0 * (quat_float[1] * quat_float[1] + quat_float[2] * quat_float[2]);
  euler[0] = atan2(sinr, cosr);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (quat_float[0] * quat_float[2] - quat_float[3] * quat_float[1]);
        if (fabs(sinp) >= 1)
            euler[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
        euler[1] = asin(sinp);

  // yaw (z-axis rotation)
  double siny = +2.0 * (quat_float[0] * quat_float[3] + quat_float[1] * quat_float[2]);
  double cosy = +1.0 - 2.0 * (quat_float[2] * quat_float[2] + quat_float[3] * quat_float[3]);  
  euler[2] = atan2(siny, cosy);
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
    
    dmp_read_fifo(gyro,accel,quat,&timestamp,&sensors,&more);

    float quat_float[4];
    float euler[3];
    quat_float[0] = quat[0]/1073741824.0;
    quat_float[1] = quat[1]/1073741824.0;
    quat_float[2] = quat[2]/1073741824.0;
    quat_float[3] = quat[3]/1073741824.0;

    quat_to_euler(quat_float, euler);



    Serial.println(euler[0]*180/M_PI);
    Serial.println(euler[1]*180/M_PI);
    Serial.println(euler[2]*180/M_PI);
  }
}
