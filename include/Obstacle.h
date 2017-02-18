#pragma once

#include <vector>
#include "Point.h"
#include <Eigen/Dense>

class Obstacle {
private:
    Eigen::Vector4d x;
    Eigen::Matrix4d P;
    Eigen::Matrix4d I;


public:
    Obstacle(double x, double y);
    predict(self,);

};