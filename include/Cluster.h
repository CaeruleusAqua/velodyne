#pragma once

#include <vector>
#include "Point.h"
#include <opencv2/imgproc/imgproc.hpp>

class Cluster {
private:


public:
    Cluster();


    std::vector<Point *> m_hull;

    unsigned int m_id;
    double m_center[3];
    bool matched;
    bool assigned;
    std::vector<Point *> m_cluster;
    cv::Point2f m_rectangle[4];

    void mean();
    void meanRect();

    double get2Distance(Cluster &a);

    double cross(const Point *O, const Point *A, const Point *B);

    void calcRectangle();



    std::vector<Point *> getHull();

    struct less_than_key {
        inline bool operator()(const Point *struct1, const Point *struct2) {

            return (struct1->getX() < struct2->getX() || (struct1->getX() == struct2->getX() && struct1->getY() < struct2->getY()));
        }
    };


    double getRectLongSite();
    double getRectShortSite();
    double getTheta();
};