class PID{
    double kp;
    double ki;
    double kd;
    double previous_error;
    double setpoint;
    unsigned long &DT;

    public:
        PID(double p, double i, double d, unsigned long &DT);
        ~PID();

        double compute(double setpoint, double measured_value);
        void setP(double p);
        void setI(double i);
        void setD(double d);
};