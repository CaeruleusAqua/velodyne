#include "Obstacle.h"
#include <math.h>
#include "Cluster.h"

LidarObstacle::LidarObstacle(double x, double y, double theta, double v, double yaw, Cluster *cluster, odcore::data::TimeStamp current_time, bool isCar) : clusterCandidates() {
    m_latestTimestamp = current_time;
    m_state << x, y, theta, v, yaw;
    m_size = cluster->m_cluster.size();
    m_isCar = isCar;

    m_rectangle[0] = cv::Point2f(cluster->m_rectangle[0]);
    m_rectangle[1] = cv::Point2f(cluster->m_rectangle[1]);
    m_rectangle[2] = cv::Point2f(cluster->m_rectangle[2]);
    m_rectangle[3] = cv::Point2f(cluster->m_rectangle[3]);
    m_initial_id = cluster->m_id;

    m_predicted << 0, 0, 0, 0, 0;

}


void LidarObstacle::predict(odcore::data::TimeStamp current_time) {
    double dt = static_cast<double>((current_time - m_latestTimestamp).toMicroseconds()) / 1000000.0;
    //std::cout << "DT: " << dt << std::endl;

    if (abs(m_state[4]) < 0.0001) {  // Driving straight
        m_predicted[0] = m_state[0] + m_state[3] * dt * cos(m_state[2]);
        m_predicted[1] = m_state[1] + m_state[3] * dt * sin(m_state[2]);
        m_predicted[2] = m_state[2];
        m_predicted[3] = m_state[3];
        m_predicted[4] = 0.0000001;//  # avoid numerical issues in Jacobians


    } else {
        m_predicted[0] = m_state[0] + (m_state[3] / m_state[4]) * (sin(m_state[4] * dt + m_state[2]) - sin(m_state[2]));
        m_predicted[1] = m_state[1] + (m_state[3] / m_state[4]) * (-cos(m_state[4] * dt + m_state[2]) + cos(m_state[2]));
        m_predicted[2] = fmod((m_state[2] + m_state[4] * dt + M_PI), (2.0 * M_PI)) - M_PI;
        m_predicted[3] = m_state[3];
        m_predicted[4] = m_state[4];

    }
}

void LidarObstacle::update(double x, double y, double theta, odcore::data::TimeStamp current_time) {
    double dt = static_cast<double>((current_time - m_latestTimestamp).toMicroseconds()) / 1000000.0;
    m_latestTimestamp = current_time;


    // calculate signed speed by rotating positions into x axis and then take x difference
    Eigen::Rotation2D<double> rot(-m_state[2]);
    Eigen::Matrix<double, 2, 1> old_state;
    old_state << m_state[0], m_state[1];
    old_state = rot.toRotationMatrix() * old_state;


    Eigen::Matrix<double, 2, 1> new_state;
    new_state << x, y;
    new_state = rot.toRotationMatrix() * new_state;

    double speed_sign = (new_state[0] - old_state[0]);
    speed_sign = speed_sign / std::abs(speed_sign);
    double speed = (std::sqrt((new_state[0] - old_state[0]) * (new_state[0] - old_state[0]) + (new_state[1] - old_state[1]) * (new_state[1] - old_state[1])) * speed_sign)/ dt;

    double yawrate = (theta - m_state[2]) / dt;


    m_state[0] = x;
    m_state[1] = y;
    m_state[2] = m_state[2] + yawrate * dt;


    m_state[3] = speed;
    m_state[4] = yawrate;

    //std::cout << "DT: " << dt << std::endl;
    //std::cout << "Speed: " << speed << std::endl;


}

double LidarObstacle::getDistance(Cluster &cluster) {
    return std::sqrt(
            (cluster.m_center[0] - m_predicted[0]) * (cluster.m_center[0] - m_predicted[0]) + (cluster.m_center[1] - m_predicted[1]) * (cluster.m_center[1] - m_predicted[1]));
}


void LidarObstacle::setIsCar(bool isCar) {
    m_isCar = isCar;
}

bool LidarObstacle::getIsCar() {
    return m_isCar;
}