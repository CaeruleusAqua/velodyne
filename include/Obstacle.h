#pragma once

#include <vector>
#include "Point.h"
#include <eigen3/Eigen/Dense>
#include "Cluster.h"
#include <list>

class LidarObstacle {
private:



public:
    Eigen::Matrix<double, 5, 1> m_x;
    Eigen::Matrix<double, 5, 1> predicted;
    Eigen::Matrix<double, 5, 5> m_P;
    Eigen::Matrix<double, 5, 5> m_I;
    Eigen::Matrix<double, 4, 4> m_R;
    double old_rot;
    double old_x;
    double old_y;
    double old_timestamp;
    double largestSingleRectLong = 0;
    double largestSingleRectSmal = 0;
    //cv::Point2f m_rectangle[4];



    LidarObstacle(double x, double y, double theta, double v, double yaw, Cluster *cluster);
    uint64_t m_initial_id;

    void predict(double dt);

    void update(Eigen::Vector4d Z);

    cv::Point2f m_rectangle[4];
    double m_center[3];
    unsigned long m_size;


    double getDistance(Cluster &cluster);
    std::list<Cluster *> clusterCandidates;
};