#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>

class IMU : Adafruit_ICM20948{
    private:
    double RawRoll,RawPitch,RawYaw;
    double GyroXOffset,GyroYOffset,GyroZOffset;
    Adafruit_ICM20948 icm;
    

    public:
    double GX,GY,GZ;
    IMU();
    void begin();
    void Update();
    double GetRoll();
    double GetPitch();
    double GetYaw();
    void CalibrateGyro();
    void CalibrateMagnoHardIron();
    // void CalibrateMagnoSoftIron();
};