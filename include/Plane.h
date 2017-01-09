#pragma once

#include <eigen3/Eigen/Dense>
#include "Point.h"
#include <vector>

class Plane {
public:
    Eigen::Vector3f normal;
    float distance = 0;

    Plane(){};
    Plane(std::vector<Point *> &points);

    float getDist(Eigen::Vector3f point);

    double fitPlaneFromPoints(std::vector<Point *> &points);
};
