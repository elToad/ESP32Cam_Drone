class PID{
    double kp;
    double ki;
    double kd;
    double previous_error;
    double setpoint;
    double integral; // Make integral a member variable to avoid static issues
    double filtered_derivative;
    double IntegralMax;
    double alpha;

    public:
        PID(double p = 0, double i = 0, double d = 0,double derivativeSmoothing = 0.5, double integralMax = 0);
        double compute(double setpoint, double measured_value,double DT);
        void setP(double p);
        void setI(double i);
        void setD(double d);
        void reset(); // Add reset function to clear integral
        void setIntegralMax(double max);
        void setSmoothing(double smooth);
};