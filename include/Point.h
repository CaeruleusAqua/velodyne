#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>

class Point {
private:
    float m_azimuth;
    float m_measurement;
    bool m_visited;
    bool m_isGround;
    Eigen::Vector3f m_point;


public:

    int m_i;
    int m_j;
    bool m_noise;
    bool m_clustered;

    Point(float x, float y, float z, float measurement, float azimuth);

    Point();

    void setIsGround(bool isGround);

    bool isGround();

    void setX(float x);

    void setY(float y);

    void setZ(float z);

    const float getX() const;

    const float getY() const;

    const float getZ() const;

    Eigen::Vector3f &getVec();

    float getAzimuth();

    float getMeasurement();

    float get2Distance(Point &a);

    void setIndex(int i, int j);

    bool isVisited();

    void setVisited(bool vis);

    bool isClustered();

    void setClustered(bool clustered);

    int getIndex();

    int getLayer();

    bool operator<(const Point &p) const {
        return (m_point[0] < p.getX() || (m_point[0] == p.getX() && m_point[1] < p.getY()));
    }

    friend std::ostream &operator<<(std::ostream &strm, Point const &point) {
        return strm << " X: " << point.m_point[0] << " Y: " << point.m_point[1] << " Z: " << point.m_point[2];
    }

};
