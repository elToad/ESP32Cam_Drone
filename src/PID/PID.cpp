#include "PID.h"

PID::PID(double p, double i, double d, unsigned long &DT):
    kp(p), ki(i), kd(d), previous_error(0), setpoint(0), DT(DT) {}

PID::~PID() {}

void PID::setP(double p) {
    kp = p;
}

void PID::setI(double i) {
    ki = i;
}
void PID::setD(double d) {
    kd = d;
}

double PID::compute(double setpoint, double measured_value) {
    double error = setpoint - measured_value;

    DT /= 1000.0; // Convert DT from milliseconds to seconds for calculation

    double P = kp * error;

    static double integral = 0;
    integral += DT;
    double I = ki * integral * DT;

    double D = kd * (error - previous_error) / (DT + 2e-16); // + small value to avoid division by zero.

    previous_error = error;
    return P + I + D;
}