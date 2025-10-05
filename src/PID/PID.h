class PID{
    double kp;
    double ki;
    double kd;
    double previous_error;
    double setpoint;
    double integral; // Make integral a member variable to avoid static issues
    double filtered_derivative;


    public:
        PID(double p, double i, double d);
        ~PID();

        double compute(double setpoint, double measured_value);
        void setP(double p);
        void setI(double i);
        void setD(double d);
        void reset(); // Add reset function to clear integral
};