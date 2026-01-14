#include "PID.h"

PID::PID(double p, double i, double d, double derivativeSmoothing, double integralMax):
    kp(p), ki(i), kd(d), previous_error(0), setpoint(0), integral(0),alpha(derivativeSmoothing) {}


void PID::setP(double p) {
    kp = p;
}

void PID::setI(double i) {
    ki = i;
}
void PID::setD(double d) {
    kd = d;
}

double PID::compute(double setpoint, double measured, double DT) {

    double error = setpoint - measured;

    double P = kp * error;
    
    integral += ki * error * DT;
    double I = integral;

    if (ki == 0) I = 0;

    if (IntegralMax != 0){
        if (I < -IntegralMax) I = -IntegralMax;
        if (I > IntegralMax) I = IntegralMax;
    }

    double error_derivative = (error - previous_error) / (DT);
    filtered_derivative = error_derivative * alpha + filtered_derivative*(1-alpha);
    double D = kd * filtered_derivative;

    previous_error = error;
    return P + I + D;
}

void PID::reset() {
    integral = 0;
}

void PID::setIntegralMax(double max){
    IntegralMax = max;
}

void PID::setSmoothing(double smooth){
    alpha = smooth;
}