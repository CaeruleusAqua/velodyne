#pragma once

#include "Point.h"
#include <eigen3/Eigen/Dense>
#include "Kalman.h"

class Kalman {




public:

    Eigen::Matrix<double, 5, 1> m_x;
    Eigen::Matrix<double, 5, 5> m_P;
    Eigen::Matrix<double, 5, 5> m_I;
    bool ready = false;

    void init(double x, double y, double theta, double v, double yaw);

    Kalman();

    void predict(double dt);

    void update(double x, double y,double theta, double speed,double yaw, double movement_x, double movement_y);

    bool isReady() {
        return ready;
    }


};


