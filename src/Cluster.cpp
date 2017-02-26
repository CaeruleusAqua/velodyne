#include "Cluster.h"
#include <algorithm>
#include <math.h>
#include <vector>
#include "Utils.h"


Cluster::Cluster() : m_cluster() {
    m_center[0] = 0;
    m_center[1] = 0;
    m_center[2] = 0;;
    m_id = 0;
    matched = false;
    assigned = false;
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


void Cluster::meanRect() {

    m_center[0] = (m_rectangle[0].x + m_rectangle[1].x + m_rectangle[2].x + m_rectangle[3].x) / 4;
    m_center[1] = (m_rectangle[0].y + m_rectangle[1].y + m_rectangle[2].y + m_rectangle[3].y) / 4;
    m_center[2] = 0;
}

double Cluster::get2Distance(Cluster &a) {
    float x = m_center[0] - a.m_center[0];
    float y = m_center[1] - a.m_center[1];
    return std::sqrt(x * x + y * y);
}


double Cluster::get2Distance(double x, double y) {
    float xx = m_center[0] - x;
    float yy = m_center[1] - y;
    return std::sqrt(xx * xx + yy * yy);
}


Point *Cluster::getMinDistPoint(double x, double y) {
    unsigned int min_dist = 200;
    Point *tmp = nullptr;
    for (auto &point : m_cluster) {
        unsigned int dist = point->get2Distance(x, y);
        if (dist < min_dist) {
            tmp = point;
            min_dist = dist;
        }
    }
    return tmp;
}

unsigned int Cluster::getSize() {
    return m_cluster.size();
}


// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
double Cluster::cross(const Point *O, const Point *A, const Point *B) {
    return (A->getX() - O->getX()) * (B->getY() - O->getY()) - (A->getY() - O->getY()) * (B->getX() - O->getX());
}


// Returns a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
std::vector<Point *> Cluster::getHull() {
    if(m_hull.empty()) {
        int n = m_cluster.size(), k = 0;
        std::vector<Point *> H(2 * n);

        // Sort points lexicographically
        std::sort(m_cluster.begin(), m_cluster.end(), less_than_key());

        // Build lower hull
        for (int i = 0; i < n; ++i) {
            while (k >= 2 && cross(H[k - 2], H[k - 1], m_cluster[i]) <= 0) k--;
            H[k++] = m_cluster[i];
        }

        // Build upper hull
        for (int i = n - 2, t = k + 1; i >= 0; i--) {
            while (k >= t && cross(H[k - 2], H[k - 1], m_cluster[i]) <= 0) k--;
            H[k++] = m_cluster[i];
        }

        H.resize(k - 1);
        m_hull = H;
        return H;
    }
    return m_hull;
}


void Cluster::calcRectangle() {
    auto hull = getHull();
    std::vector<cv::Point2f> vec;
    for (auto &point : hull) {
        vec.push_back(cv::Point2f(point->getX(), point->getY()));
    }
    if (vec.size() > 2) {
        auto rect = cv::minAreaRect(vec);
        rect.points(m_rectangle);
    }
}

double Cluster::getTheta() {
    double lenA = cv::norm(m_rectangle[0] - m_rectangle[1]);
    double lenB = cv::norm(m_rectangle[1] - m_rectangle[2]);
    // find long site
    if (lenA >= lenB)
        // make sure taking pint , i'm unsure that this is enough
        if (m_rectangle[0].y < m_rectangle[1].y) {
            return std::atan2(m_rectangle[1].y - m_rectangle[0].y, m_rectangle[1].x - m_rectangle[0].x);
        } else {
            return std::atan2(m_rectangle[0].y - m_rectangle[1].y, m_rectangle[0].x - m_rectangle[1].x);
        }
    else {
        if (m_rectangle[1].y < m_rectangle[2].y) {
            return std::atan2(m_rectangle[2].y - m_rectangle[1].y, m_rectangle[2].x - m_rectangle[1].x);
        } else {
            return std::atan2(m_rectangle[1].y - m_rectangle[2].y, m_rectangle[1].x - m_rectangle[2].x);
        }
    }
}


double Cluster::getRectShortSite() {
    double lenA = cv::norm(m_rectangle[0] - m_rectangle[1]);
    double lenB = cv::norm(m_rectangle[1] - m_rectangle[2]);
    if (lenA < lenB)
        return lenA;
    else
        return lenB;
}

double Cluster::getRectLongSite() {
    double lenA = cv::norm(m_rectangle[0] - m_rectangle[1]);
    double lenB = cv::norm(m_rectangle[1] - m_rectangle[2]);
    if (lenA >= lenB)
        return lenA;
    else
        return lenB;
}