//
// Created by storm on 23.11.16.
//

#include "Point.h"
#include <cmath>


Point::Point(float x, float y, float z, float azimuth, float measurement) {
    m_pos[0] = x;
    m_pos[1] = y;
    m_pos[2] = z;
    m_azimuth = azimuth;
    m_measurement = measurement;
    m_visited = false;
    m_noise = false;
    clustered = false;
}

void Point::setX(float x) {
    m_pos[0] = x;
}

void Point::setY(float y) {
    m_pos[1] = y;
}

void Point::setZ(float z) {
    m_pos[2] = z;
}

float *Point::getPos() {
    return m_pos;
}

float Point::getAzimuth() {
    return m_azimuth;
}

float Point::getMeasurement() {
    return m_measurement;
}

float Point::get2Distance(Point a) {
    float x = getPos()[0] - a.getPos()[0];
    float y = getPos()[1] - a.getPos()[1];
    return std::sqrt(x * x + y * y);
}

bool Point::isVisited() {
    return m_visited;
}

void Point::setVisited(bool vis) {
    m_visited = vis;
}

void Point::setIndex(int i, int j) {
    m_i = i;
    m_j = j;
}

int Point::getI() {
    return m_i;
}

int Point::getJ() {
    return m_j;
}

std::ostream &operator<<(std::ostream &strm, const Point &p) {
    float x = p.m_pos[0];
    float y = p.m_pos[1];
    float z = p.m_pos[2];

    return strm << " X: " << x << " Y: " << y << " Z: " << z;
}

