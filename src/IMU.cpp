#include "IMU.h"
#include <Wire.h>

IMU::IMU(int SDA,int SCL, int clock = 400000) : SDA(SDA), SCL(SCL), Clock(clock) {
  icm.begin_I2C();
  icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_1000_DPS);
  icm.setMagDataRate(ak09916_data_rate_t::AK09916_MAG_DATARATE_100_HZ);

  icm.setAccelRateDivisor(3);
  icm.setGyroRateDivisor(3);
};

void IMU::Update(){
  sensors_event_t acc, gyro, mag, temp;
  icm.getEvent(&acc, &gyro, &temp, &mag);
  
  double GX = gyro.gyro.x - GyroXOffset;
  double GY = gyro.gyro.y - GyroYOffset;
  double GZ = gyro.gyro.z - GyroZOffset;
  
  double AX = acc.acceleration.x;
  double AY = acc.acceleration.y;
  double AZ = acc.acceleration.z;
  
  // Apply magnetometer calibration offsets
  double MX = mag.magnetic.x; 
  double MY = mag.magnetic.y;
  double MZ = mag.magnetic.z;

  //My imu is rotated so Y is pitch and X is roll.
  // Calculate angles from accelerometer (in radians)
  double rawPitch = atan(AY/(sqrt(pow(AX,2) + pow(AZ,2))));
  double rawRoll = -atan(AX/(sqrt(pow(AY,2) + pow(AZ,2))));
}

double IMU::GetRoll(){
    return RawRoll;
}

double IMU::GetPitch(){
    return RawPitch;
}

double IMU::GetYaw(){
    return RawYaw;
}

void IMU::CalibrateMagno(){
      for (int i = 0; i < 2000; i++){
    sensors_event_t acc, gyro, mag, temp;
    icm.getEvent(&acc, &gyro, &temp, &mag);
    }
}

void IMU::CalibrateGyro(){
  Serial.println("Calibrating gyro");
  Serial.println("Keep the IMU stationary for gyro calibration");
  
  // Variables for magnetometer calibration (min/max method)
  double magXMin = 1000, magXMax = -1000;
  double magYMin = 1000, magYMax = -1000;
  double magZMin = 1000, magZMax = -1000;
  
  for (int i = 0; i < 2000; i++){
    sensors_event_t acc, gyro, mag, temp;
    icm.getEvent(&acc, &gyro, &temp, &mag);
    
    // Gyroscope calibration (first 1000 samples - keep still)
    if (i < 1000) {
      GyroXOffset += gyro.gyro.x;
      GyroYOffset += gyro.gyro.y;
      GyroZOffset += gyro.gyro.z;
    }
    
    // Magnetometer calibration (all samples - can rotate after first 1000)
    if (mag.magnetic.x < magXMin) magXMin = mag.magnetic.x;
    if (mag.magnetic.x > magXMax) magXMax = mag.magnetic.x;
    if (mag.magnetic.y < magYMin) magYMin = mag.magnetic.y;
    if (mag.magnetic.y > magYMax) magYMax = mag.magnetic.y;
    if (mag.magnetic.z < magZMin) magZMin = mag.magnetic.z;
    if (mag.magnetic.z > magZMax) magZMax = mag.magnetic.z;

    if (i == 1000) {
      Serial.println("Gyro calibration Complete");
    }
    
    delay(1);
    }
}