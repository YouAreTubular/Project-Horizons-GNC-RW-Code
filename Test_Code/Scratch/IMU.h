#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Adafruit_LSM6DSO32.h>  //install Adafruit LSM6DS library

struct IMUData {
  uint32_t timestamp;
  
  float accelX;
  float accelY;
  float accelZ;

  float gyroX;
  float gyroY;
  float gyroZ;

  float temperature;
};

class IMU {
public:
  bool begin();
  IMUData read();

private:
  Adafruit_LSM6DSO32 lsm;
};

#endif