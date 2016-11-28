#pragma once

#include <iostream>

class Point {
private:
    float m_x,m_y,m_z;
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

    friend std::ostream &operator<<(std::ostream &outputStream, const Point &p);

    void setX(float x);

    void setY(float y);

    void setZ(float z);

    float getX();

    float getY();

    float getZ();

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
};
