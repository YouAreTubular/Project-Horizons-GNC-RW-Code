#include "IMU.h"

bool IMU::begin() {

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();

  if (!lsm.begin_I2C()) {
    return false;
  }

  // Accelerometer settings
  lsm.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);

  // Gyroscope settings
  lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

  // Output data rate
  lsm.setAccelDataRate(LSM6DS_RATE_416_HZ);
  lsm.setGyroDataRate(LSM6DS_RATE_416_HZ);

  return true;
}

IMUData IMU::read() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;



  lsm.getEvent(&accel, &gyro, &temp);

  IMUData data;

  data.timestamp = millis();

  data.accelX = accel.acceleration.x;
  data.accelY = accel.acceleration.y;
  data.accelZ = accel.acceleration.z;

  data.gyroX = gyro.gyro.x;
  data.gyroY = gyro.gyro.y;
  data.gyroZ = gyro.gyro.z;

  data.temperature = temp.temperature;

  return data;
}