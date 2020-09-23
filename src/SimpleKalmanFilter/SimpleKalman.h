//
// Created by ajahueym on 22/09/20.
//

#ifndef KALMAN_SIMPLEKALMAN_H
#define KALMAN_SIMPLEKALMAN_H
#include <eigen3/Eigen/Dense>

class SimpleKalman {
public:
    SimpleKalman(double initialPos, double initialVel, double accelVariance);
    void predict(double timeStep);
    void update(double measurement, double measurementVariance);
    const Eigen::MatrixXd &getState() const;
    const Eigen::MatrixXd &getCovariance() const;
private:
    double accelVariance;
    Eigen::MatrixXd state {2, 1};
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(2, 2);

    Eigen::MatrixXd F {2, 2};
    Eigen::MatrixXd G {2, 1};
    Eigen::MatrixXd H {1, 2};
};

#endif //KALMAN_SIMPLEKALMAN_H
