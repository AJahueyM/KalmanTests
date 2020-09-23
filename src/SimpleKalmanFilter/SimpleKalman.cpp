//
// Created by ajahueym on 22/09/20.
//

#include "SimpleKalman.h"
#include <iostream>

SimpleKalman::SimpleKalman(double initialPos, double initialVel,  double accelVariance) {
    this->accelVariance = accelVariance;
    state << initialPos, initialVel;
}

void SimpleKalman::predict(double timeStep) {
    F = Eigen::MatrixXd::Identity(2, 2);
    G = Eigen::MatrixXd::Zero(2, 1);

    F <<    1,  timeStep,
            0,  1;
    G <<    0.5 * std::pow(timeStep, 2),
            timeStep;

    state = F * state;

    covariance = F * covariance * F.transpose() + G * G.transpose() * accelVariance ;
}

void SimpleKalman::update(double measurement, double measurementVariance) {
    H = Eigen::MatrixXd::Zero(1, 2);
    H << 1, 0;

    Eigen::MatrixXd z{1, 1}, r {1, 1};
    z << measurement;
    r << measurementVariance;

    Eigen::MatrixXd y = z - H * state;
    Eigen::MatrixXd s = H * covariance * H.transpose() + r;

    Eigen::MatrixXd k = covariance * H.transpose() * s.inverse();

    state = state + k * y;
    covariance = (Eigen::MatrixXd::Identity(2, 2) - k * H) * covariance;
}

const Eigen::MatrixXd &SimpleKalman::getState() const {
    return state;
}

const Eigen::MatrixXd &SimpleKalman::getCovariance() const {
    return covariance;
}

