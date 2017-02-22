#pragma once

#include <vector>
#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include "Point.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "Cluster.h"
#include <list>

class LidarObstacle {
private:

    odcore::data::TimeStamp m_latestTimestamp;


public:
    Eigen::Matrix<double, 5, 1> m_state;
    Eigen::Matrix<double, 5, 1> m_predicted;


    LidarObstacle(double x, double y, double theta, double v, double yaw, Cluster *cluster, odcore::data::TimeStamp current_time);

    uint64_t m_initial_id;

    void predict(odcore::data::TimeStamp current_time);

    void update(double x, double y, double theta, odcore::data::TimeStamp current_time);

    cv::Point2f m_rectangle[4];
    unsigned long m_size;


    double getDistance(Cluster &cluster);

    std::list<Cluster *> clusterCandidates;
};