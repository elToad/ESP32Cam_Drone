class KalmanCompFilter{
    private:
    double Kalman = 0;
    double KalmanUncertainty{}, KalmanOutput{}, KalmanState{};

    public:
    KalmanCompFilter(double Uncertainty);
    double Compute(double KalmanInput, double KalmanMeasurement, double DT);
};