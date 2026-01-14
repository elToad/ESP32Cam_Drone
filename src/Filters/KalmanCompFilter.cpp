#include <cmath>
#include "KalmanCompFilter.h"

KalmanCompFilter::KalmanCompFilter(double Uncertainty = 2) : KalmanUncertainty(Uncertainty){};

double KalmanCompFilter::Compute(double KalmanInput, double KalmanMeasurement,double DT){

  KalmanState = KalmanState + DT * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + pow(DT,2) * pow(4,2);

  float KalmanGain = KalmanUncertainty * 1/ (1*KalmanUncertainty + pow(3,2));
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1-KalmanGain)*KalmanUncertainty;

  return KalmanState;
}