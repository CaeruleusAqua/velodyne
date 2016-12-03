#pragma once

#include <vector>
#include "Point.h"

class Cluster {
private:


public:
    Cluster();

    unsigned int m_id;
    double m_center[3];
    bool matched;
    std::vector<Point *> m_cluster;

    void mean();

    double get2Distance(Cluster &a);
};