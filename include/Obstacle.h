#pragma once

#include <vector>
#include "Point.h"
#include <eigen3/Eigen/Dense>

class LidarObstacle {
private:
    Eigen::Matrix<double, 5, 1> m_x;
    Eigen::Matrix<double, 5, 5> m_P;
    Eigen::Matrix<double, 5, 5> m_I;
    Eigen::Matrix<double, 4, 4> m_R;


public:
    LidarObstacle(double x, double y, double theta, double v, double yaw);

    void predict(double dt);

    void update(Eigen::Vector4d Z);
};