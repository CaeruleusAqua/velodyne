#include "Cluster.h"
#include <algorithm>
#include <cmath>
#include <vector>


Cluster::Cluster() {
    m_center[0] = 0;
    m_center[1] = 0;
    m_center[2] = 0;;
    m_id = 0;
    matched = false;
}

void Cluster::mean() {
    m_center[0] = 0;
    m_center[1] = 0;
    m_center[2] = 0;
    for (auto point : m_cluster) {
        m_center[0] += point->getX();
        m_center[1] += point->getY();
        m_center[2] += point->getZ();
    }
    m_center[0] /= m_cluster.size();
    m_center[1] /= m_cluster.size();
    m_center[2] /= m_cluster.size();

}

double Cluster::get2Distance(Cluster &a) {
    float x = m_center[0] - a.m_center[0];
    float y = m_center[1] - a.m_center[1];
    return std::sqrt(x * x + y * y);
}