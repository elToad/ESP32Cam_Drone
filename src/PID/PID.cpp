#include "PID.h"
#include "Arduino.h"

PID::PID(double p, double i, double d):
    kp(p), ki(i), kd(d), previous_error(0), setpoint(0), integral(0) {}

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

double PID::compute(double setpoint, double measured) {

    double error = setpoint - measured;

    double P = kp * error;
    
    integral += ki * error * 0.004;
    double I = integral;

    if (ki == 0) I = 0;

    double alpha = 0.42;
    double error_derivative = (error - previous_error) / (0.004);
    filtered_derivative = error_derivative * alpha + filtered_derivative*(1-alpha);
    double D = kd * filtered_derivative;

    previous_error = error;
    return P + I + D;
}

void PID::reset() {
    integral = 0;
    //previous_error = 0;
}