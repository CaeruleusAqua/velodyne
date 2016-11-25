#pragma once

#include <iostream>

class Point {
private:
    float m_pos[3];
    float m_azimuth;
    float m_measurement;
    bool m_visited;


public:

    int m_i;
    int m_j;
    bool m_noise;
    bool clustered;

    Point(float x, float y, float z, float azimuth, float measurement);

    friend std::ostream &operator<<(std::ostream &outputStream, const Point &p);

    void setX(float x);

    void setY(float y);

    void setZ(float z);

    float *getPos();

    float getAzimuth();

    float getMeasurement();

    float get2Distance(Point a);

    void setIndex(int i, int j);

    bool isVisited();

    void setVisited(bool vis);

    int getI();
    int getJ();
};
