#include <iostream>
#include "matplotlibcpp.h"
#include "SimpleKalmanFilter/SimpleKalman.h"
namespace plt = matplotlibcpp;

double rand_gen() {
    // return a uniformly distributed random value
    return ( (double)(rand()) + 1. )/( (double)(RAND_MAX) + 1. );
}
double normalRandom() {
    // return a normally distributed random value
    double v1=rand_gen();
    double v2=rand_gen();
    return cos(2*3.14*v2)*sqrt(-2.*log(v1));
}

int main() {

    double realX = 0;
    double realVel = 0.5;

    SimpleKalman kf(realX,  realVel, 0.1);

    const double dT = 0.1;
    const int steps = 10000;
    const int stepsPerMeasurement = 20;
    const double measurementVariance = std::pow(0.1, 2);


    std::vector<double> realXs, estimatedXs, noisyXs, readings;
    std::vector<double> varianceNegative, variancePositive;
    double noisyX = 0;

    for(int i = 0; i < steps; ++i){

        if(i > 500){
            realVel *= 0.9;
        }
        realX = realX + realVel * dT;

        double noisyVel = realVel + normalRandom() * std::sqrt(measurementVariance);

        noisyX = noisyX + noisyVel * dT;
        kf.predict(dT);

        double noisyReading  = realX + normalRandom() * std::sqrt(measurementVariance);
        if(i != 0 && i % stepsPerMeasurement == 0){
            kf.update(noisyReading, measurementVariance);
        }

        auto estimation = kf.getState();
        auto covariance = kf.getCovariance();

        realXs.emplace_back(realX);
        noisyXs.emplace_back(noisyX);
        estimatedXs.emplace_back(estimation(0,0));
        readings.emplace_back(noisyReading);

        variancePositive.emplace_back(estimation(0,0) + 2 * std::sqrt(covariance(0,0)));
        varianceNegative.emplace_back(estimation(0,0) - 2 * std::sqrt(covariance(0,0)));

    }

    plt::title("Position");
    plt::plot(realXs, "b");
    plt::plot(readings, "b--");
    plt::plot(estimatedXs, "r");
    plt::plot(variancePositive, "r--");
    plt::plot(varianceNegative, "r--");
    plt::plot(noisyXs, "g");
    plt::show();
}