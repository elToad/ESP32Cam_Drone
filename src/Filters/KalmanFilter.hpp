#include <ArduinoEigen.h>

class KalmanFilter{
    public:
    KalmanFilter();
    ~KalmanFilter();

    void Predict(Eigen::Vector3d gyro, double dt);
    void Update(Eigen::Vector3d z);

    Eigen::Vector3d GetStates();

    private:
    Eigen::Vector3d x;
    Eigen::Matrix3d P;
    Eigen::Matrix3d Q;
    Eigen::Matrix3d R;
};