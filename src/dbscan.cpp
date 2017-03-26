#include "dbscan.h"
#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include "opendavinci/odcore/wrapper/half_float.h"
#include <opencv2/core/core.hpp>
#include <chrono>


void DbScan::getClusters(std::vector<Cluster> &clusters) {
    for (uint32_t angle = 0; angle < m_cloudSize; angle++) {
        for (int index = 0; index < 16; index++) {
            Point *point = &m_points[angle][index];
            if (!point->isVisited() && !point->isClustered() &&!point->isGround()) {
                point->setVisited(true);
                auto neighbors = std::vector<Point *>();
                regionQuery(neighbors, point);
                if (neighbors.size() > m_minPts) {
                    clusters.push_back(Cluster());
                    clusters.back().m_cluster.push_back(point);
                    point->setClustered(true);
                    expandCluster(neighbors, clusters.back());
                }
            }
        }
    }
}


void DbScan::regionQuery(std::vector<Point *> &neighbors, Point *point) {
    int didx = 5;
    int i = point->getIndex();
    int neg_idx = i - didx;
    int pos_idx = i + 1 + didx - m_cloudSize;


    for (uint32_t k = m_cloudSize + neg_idx; k < m_cloudSize; k++) {
        for (int l = 0; l < 16; l++) {
            if (point->get2Distance(m_points[k][l]) < m_eps) {
                neighbors.push_back(&m_points[k][l]);
            }

        }
    }

    for (int k = 0; k < pos_idx; k++) {
        for (int l = 0; l < 16; l++) {
            if (point->get2Distance(m_points[k][l]) < m_eps) {
                neighbors.push_back(&m_points[k][l]);
            }

        }
    }

    for (int k = std::max(0, neg_idx); k < std::min(i + 1 + didx, (int) m_cloudSize); k++) {
        for (int l = 0; l < 16; l++) {
            if (point->get2Distance(m_points[k][l]) < m_eps) {
                neighbors.push_back(&m_points[k][l]);
            }

        }
    }

}


void DbScan::expandCluster(std::vector<Point *> &neighbors, Cluster &cluster) {
    while (!neighbors.empty()) {
        Point *point = neighbors.back();
        neighbors.pop_back();

        if (!point->isVisited()) {
            point->setVisited(true);
            auto collection = std::vector<Point *>();
            if(!point->isGround())
                regionQuery(collection, point);
            if (collection.size() > m_minPts) {
                neighbors.insert(neighbors.end(), collection.begin(), collection.end());
            }
        }
        if (!point->isClustered()) {
            cluster.m_cluster.push_back(point);
            point->setClustered(true);
        }
    }
}