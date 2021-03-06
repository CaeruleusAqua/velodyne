#pragma once

#include <vector>
#include "Point.h"
#include <opencv2/imgproc/imgproc.hpp>

class Cluster {
private:


public:
    Cluster();


    std::vector<Point *> m_hull;

    double m_center[3];
    bool assigned;
    std::vector<Point *> m_cluster;
    cv::Point2f m_rectangle[4];

    void mean();
    void meanRect();

    double get2Distance(Cluster &a);
    double get2Distance(double x ,double y);
    unsigned int getSize();

    Point* getMinDistPoint(double x ,double y);



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