//
// Created by storm on 23.11.16.
//

#include "Point.h"
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>


Point::Point(float x, float y, float z, float azimuth, float measurement) {
    m_x = x;
    m_y = y;
    m_z = z;
    m_azimuth = azimuth;
    m_measurement = measurement;
    m_visited = false;
    m_noise = false;
    m_clustered = false;
}


Point::Point() {
    m_visited = false;
    m_noise = false;
    m_clustered = false;
}

Eigen::Vector3f Point::getVec(){
    return Eigen::Vector3f(m_x,m_y,m_z);
}

void Point::setX(float x) {
    m_x = x;
}

void Point::setY(float y) {
    m_y = y;
}

void Point::setZ(float z) {
    m_z = z;
}

const float Point::getX() const{
    return m_x;
}

const float Point::getY() const{
    return m_y;
}

const float Point::getZ() const{
    return m_z;
}

float Point::getAzimuth() {
    return m_azimuth;
}

float Point::getMeasurement() {
    return m_measurement;
}

float Point::get2Distance(Point &a) {
    float x = m_x - a.getX();
    float y = m_y - a.getY();
    return std::sqrt(x * x + y * y);
}

bool Point::isVisited() {
    return m_visited;
}

void Point::setVisited(bool vis) {
    m_visited = vis;
}


bool Point::isClustered() {
    return m_clustered;
}

void Point::setClustered(bool clustered) {
    m_clustered = clustered;
}

void Point::setIndex(int i, int j) {
    m_i = i;
    m_j = j;
}

int Point::getIndex() {
    return m_i;
}

int Point::getLayer() {
    return m_j;
}

