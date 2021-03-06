#pragma once

#include "Utils.h"
#include "Point.h"
#include <iostream>
#include "Cluster.h"


class DbScan {
public:
    void getClusters(std::vector<Cluster> &clusters);

    DbScan(Point (&points)[2000][16], unsigned int cloudSize)
            : m_points(points), m_cloudSize(cloudSize) {
    };

    void setDataReference(Point (&array)[2000][16], unsigned int cloudSize);

private:


    void regionQuery(std::vector<Point *> &collection, Point *point);

    void expandCluster(std::vector<Point *> &neighbors, Cluster &cluster);

    Point (&m_points)[2000][16];
    unsigned int m_cloudSize;
    static constexpr float m_eps = 1.8;
    static constexpr uint32_t m_minPts = 5;
};

