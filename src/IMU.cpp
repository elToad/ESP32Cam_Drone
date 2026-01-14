#include "IMU.h"
#include <Wire.h>
#include <ArduinoEigen.h>
#include <Arduino.h>

IMU::IMU() {
  // Initialize variables but don't start I2C communication yet
  GyroXOffset = 0;
  GyroYOffset = 0;
  GyroZOffset = 0;
  RawRoll = 0;
  RawPitch = 0;
  RawYaw = 0;
  GX = 0;
  GY = 0;
  GZ = 0;
}

void IMU::begin() {
  
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip");
    return;
  }
  
  icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_1000_DPS);
  icm.setMagDataRate(ak09916_data_rate_t::AK09916_MAG_DATARATE_100_HZ);

  icm.setAccelRateDivisor(3);
  icm.setGyroRateDivisor(3);
  
  Serial.println("ICM20948 initialized successfully");
};

void IMU::Update(){
  sensors_event_t acc, gyro, mag, temp;
  
  // Check if IMU is properly initialized before reading
  if (!icm.getEvent(&acc, &gyro, &temp, &mag)) {
    Serial.println("Failed to read IMU data");
    return;
  }
  
  GX = gyro.gyro.x - GyroXOffset;
  GY = gyro.gyro.y - GyroYOffset;
  GZ = gyro.gyro.z - GyroZOffset;
  
  double AX = acc.acceleration.x;
  double AY = acc.acceleration.y;
  double AZ = acc.acceleration.z;

  Eigen::Vector3d Mag(mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  const Eigen::Vector3d HardIron(2.0, -26.02, 33.83);
  const Eigen::Matrix3d SoftIron = (Eigen::Matrix3d() << 
    1.040, -0.025, 0.014,
    -0.025, 0.969, 0.013,
    0.014, 0.013, 0.993).finished();

  Mag =  SoftIron * (Mag - HardIron);

  //My imu is rotated so Y is pitch and X is roll.
  //Calculate angles from accelerometer (in radians)
  RawPitch = atan(AY/(sqrt(pow(AX,2) + pow(AZ,2))));
  RawRoll = -atan(AX/(sqrt(pow(AY,2) + pow(AZ,2))));
  //Magnometer Heading/Yaw
  RawYaw = -atan(Mag(0)/Mag(1));
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
    
    if (i == 1000) {
      GyroXOffset /= 1000;
      GyroYOffset /= 1000;
      GyroZOffset /= 1000;
      Serial.println("Gyro calibration Complete");
    }
    
    delay(1);
    }
}