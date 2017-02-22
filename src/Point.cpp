//
// Created by storm on 23.11.16.
//

#include "Point.h"
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>


Point::Point(float x, float y, float z, float measurement, float azimuth) : m_point(x, y, z) {

    m_azimuth = azimuth;
    m_measurement = measurement;
    m_visited = false;
    m_noise = false;
    m_clustered = false;
    m_isGround = false;
}


Point::Point() {
    m_visited = false;
    m_noise = false;
    m_clustered = false;
}

Eigen::Vector3f &Point::getVec() {
    return m_point;
}

void Point::setX(float x) {
    m_point[0] = x;
}

void Point::setIsGround(bool isGround) {
    m_isGround = isGround;
}

bool Point::isGround() {
    return m_isGround;
}

void Point::setY(float y) {
    m_point[1] = y;
}

void Point::setZ(float z) {
    m_point[2] = z;
}

const float Point::getX() const {
    return m_point[0];
}

const float Point::getY() const {
    return m_point[1];
}

const float Point::getZ() const {
    return m_point[2];
}

float Point::getAzimuth() {
    return m_azimuth;
}

float Point::getMeasurement() {
    return m_measurement;
}

float Point::get2Distance(Point &a) {
    float x = m_point[0] - a.getX();
    float y = m_point[1] - a.getY();

    return std::sqrt(x * x + y * y);

   // return (m_point-a.getVec()).norm();
}


float Point::get2Distance(float x, float y) {
    float tx = m_point[0] - x;
    float ty = m_point[1] - y;

    return std::sqrt(tx * tx + ty * ty);
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

