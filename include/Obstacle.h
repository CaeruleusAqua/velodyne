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
    double m_boxLongSite=0;
    double m_boxShortSite = 0;
public:
    double getBoxLongSite() const;

    void setBoxLongSite(double boxLongSite);

    double getBoxShortSite() const;

    void setBoxShortSite(double boxShortSite);

    double getOldBoxLongSite() const;

    void setOldBoxLongSite(double oldBoxLongSite);

    double getOldBoxShortSite() const;

    void setOldBoxShortSite(double oldBoxShortSite);

private:
    double oldBoxLongSite=0;
    double oldBoxShortSite=0;


public:

    int32_t lostTrackingCounts = 0;
    Eigen::Matrix<double, 5, 1> m_state;
    Eigen::Matrix<double, 5, 1> m_predicted;
    cv::Point2f m_rectangle[4];
    uint64_t m_initial_id;
    std::list<Cluster *> clusterCandidates;


    LidarObstacle(double x, double y, double theta, double v, double yaw, Cluster *cluster, odcore::data::TimeStamp current_time);
    void predict(odcore::data::TimeStamp current_time);
    void update(double x, double y, double theta, odcore::data::TimeStamp current_time);
    double getDistance(Cluster &cluster);

};