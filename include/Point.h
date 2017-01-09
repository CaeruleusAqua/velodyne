#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>

class Point {
private:
    float m_x, m_y, m_z;
    float m_azimuth;
    float m_measurement;
    bool m_visited;


public:

    int m_i;
    int m_j;
    bool m_noise;
    bool m_clustered;

    Point(float x, float y, float z, float azimuth, float measurement);

    Point();

    void setX(float x);

    void setY(float y);

    void setZ(float z);

    const float getX() const;

    const float getY() const;

    const float getZ() const;

    Eigen::Vector3f getVec();

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
        return (m_x < p.getX() || (m_x == p.getX() && m_y < p.getY()));
    }

    friend std::ostream &operator<<(std::ostream &strm, Point const & point) {
        return strm << " X: " << point.m_x << " Y: " << point.m_y << " Z: " << point.m_z;
    }

};