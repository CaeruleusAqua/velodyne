#pragma once

#include "Point.h"
#include <list>


class Pointcloud {
public:
    Pointcloud();
    void addPoint(Point &point);
    std::list<Point> m_points[200][200];
    std::list<Point *> getPointsNextTo(float x, float y , float dist);
    void clean();
private:




};

