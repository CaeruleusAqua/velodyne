#include "dbscan.h"
#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include "opendavinci/odcore/wrapper/half_float.h"
#include <opencv2/core/core.hpp>
#include <chrono>


void DbScan::getClusters(std::vector<std::vector<Point *>> &clusters) {
    for (int angle = 0; angle < m_cloudSize; angle++) {
        for (int index = 0; index < 16; index++) {
            Point *point = &m_points[angle][index];
            if (!point->isVisited() && !point->isClustered()) {
                point->setVisited(true);
                auto neighbors = std::vector<Point *>();
                regionQuery(neighbors, point);
                if (neighbors.size() > m_minPts) {
                    clusters.push_back(std::vector<Point *>());
                    clusters.back().push_back(point);
                    point->setClustered(true);
                    expandCluster(neighbors, clusters.back());
                }
            }
        }
    }
}


void DbScan::regionQuery(std::vector<Point *> &neighbors, Point *point) {
    int didx = 2;
    int i = point->getIndex();
    for (int k = std::max(0, i - didx); k < std::min(i + 1 + didx, (int) m_cloudSize); k++) {
        for (int l = 0; l < 16; l++) {
            if (point->get2Distance(m_points[k][l]) < m_eps) {
                neighbors.push_back(&m_points[k][l]);
            }

        }
    }
}


void DbScan::expandCluster(std::vector<Point *> &neighbors, std::vector<Point *> &cluster) {
    while (!neighbors.empty()) {
        Point *point = neighbors.back();
        neighbors.pop_back();

        if (!point->isVisited()) {
            point->setVisited(true);
            auto collection = std::vector<Point *>();
            regionQuery(collection, point);
            if (collection.size() > m_minPts) {
                neighbors.insert(neighbors.end(), collection.begin(), collection.end());
            }
        }
        if (!point->isClustered()) {
            cluster.push_back(point);
            point->setClustered(true);
        }
    }
}