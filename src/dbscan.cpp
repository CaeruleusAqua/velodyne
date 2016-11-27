#include "dbscan.h"
#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include "opendavinci/odcore/wrapper/half_float.h"
#include <opencv2/core/core.hpp>
#include <chrono>
#include <algorithm>

void DbScan::getClusters(std::vector<std::vector<Point *>> &clusters) {
    for (int i = 0; i < m_cloudSize; i++) {
        for (int j = 0; j < 16; j++) {
            Point *point = &m_points[i][j];
            if (!point->isVisited()) {
                point->setVisited(true);
                std::vector<Point *> collection = regionQuery(point);
                if (collection.size() > m_minPts) {
                    std::vector<Point *> cluster;
                    cluster.push_back(point);
                    point->clustered = true;
                    expandCluster(collection, cluster);
                    clusters.push_back(cluster);
                }

            }
        }
    }
}


std::vector<Point *> DbScan::regionQuery(Point *point) {
    int didx = 3;
    std::vector<Point *> collection;
    //float distance = point->getMeasurement();
    int i = point->getI();
    for (int k = std::max(0, i - didx); k < std::min(i + didx, (int) m_cloudSize); k++) {
        for (int l = 0; l < 16; l++) {
            float z = point->getPos()[2];
            if (0 > z && z > -2) {
                if (point->get2Distance(m_points[k][l]) < m_eps) {
                    collection.push_back(&m_points[k][l]);
                }
            }
        }

    }
    return collection;
}


void DbScan::expandCluster(std::vector<Point *> &neighbors, std::vector<Point *> &cluster) {
    while (!neighbors.empty()) {
        Point *point = neighbors.back();
        neighbors.pop_back();

        if (!point->isVisited()) {
            point->setVisited(true);
            std::vector<Point *> collection = regionQuery(point);
            if (collection.size() > m_minPts) {
                neighbors.insert(neighbors.end(), collection.begin(), collection.end());
            }
        }
        if (!point->clustered) {
            cluster.push_back(point);
            point->clustered = true;
        }
    }
}