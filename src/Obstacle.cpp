#include "Obstacle.h"
#include <math.h>
#include <list>



LidarObstacle::LidarObstacle(double x, double y, double theta, double v, double yaw, Cluster *cluster, odcore::data::TimeStamp current_time) : clusterCandidates() {
    m_latestTimestamp = current_time;
    m_state << x, y, theta, v, yaw;

    m_rectangle[0] = cv::Point2f(cluster->m_rectangle[0]);
    m_rectangle[1] = cv::Point2f(cluster->m_rectangle[1]);
    m_rectangle[2] = cv::Point2f(cluster->m_rectangle[2]);
    m_rectangle[3] = cv::Point2f(cluster->m_rectangle[3]);
    m_initial_id = cluster->m_id;

    m_predicted << 0, 0, 0, 0, 0;

}


LidarObstacle::LidarObstacle(Cluster *cluster, odcore::data::TimeStamp current_time) : clusterCandidates() {
    m_latestTimestamp = current_time;
    m_state << cluster->m_center[0], cluster->m_center[1], 0, 0, 0;

    m_initial_id = cluster->m_id;

    m_predicted << 0, 0, 0, 0, 0;

}


void LidarObstacle::refresh(double movement_x, double movement_y, odcore::data::TimeStamp current_time) {


    if (clusterCandidates.size() > 0) {
        Eigen::Rotation2D<float> rot(-m_state[2]);
        std::list<Eigen::Vector2f> points;

        for (auto &cluster : clusterCandidates) {
            for (auto &point : cluster->getHull()) {
                Eigen::Vector2f old_point;
                old_point << point->getX(), point->getY();
                points.push_back(rot.toRotationMatrix() * old_point);
            }
        }

        float min_x = points.front()[0];
        float max_x = points.back()[0];
        float min_y = points.front()[1];
        float max_y = points.front()[1];
        for (auto &vec : points) {
            if (vec[1] < min_y)
                min_y = vec[1];
            if (vec[1] > max_y)
                max_y = vec[1];
            if (vec[0] > max_x)
                max_x = vec[0];
            if (vec[0] < min_x)
                min_x = vec[0];
        }

        float dxdd = (max_x - min_x) / 3.0f;
        float x1 = min_x + dxdd;
        float x2 = min_x + dxdd * 2;

        float p1_x = FLT_MAX;
        float p1_y = FLT_MAX;
        float p2_x = FLT_MAX;
        float p2_y = FLT_MAX;

        for (auto &vec : points) {
            if (vec[0] < x1) {
                if (vec[1] < p1_y) {
                    p1_x = vec[0];
                    p1_y = vec[1];
                }
            }
            if (vec[0] > x2) {
                if (vec[1] < p2_y) {
                    p2_x = vec[0];
                    p2_y = vec[1];
                }

            }
        }


        float thetaCorrection = std::atan2(p2_y-p1_y, p2_x-p1_x);
        Eigen::Rotation2D<float> rotCorrection(-thetaCorrection);




        for(auto &point : points){
            point = rotCorrection.toRotationMatrix() * point;
        }


        min_x = points.front()[0];
        max_x = points.back()[0];
        min_y = points.front()[1];
        max_y = points.front()[1];
        for (auto &vec : points) {
            if (vec[1] < min_y)
                min_y = vec[1];
            if (vec[1] > max_y)
                max_y = vec[1];
            if (vec[0] > max_x)
                max_x = vec[0];
            if (vec[0] < min_x)
                min_x = vec[0];
        }



        float height = max_y - min_y;

        Eigen::Vector2f newPos(max_x, max_y);
        Eigen::Rotation2D<float> rotBack(m_state[2]+thetaCorrection);
        newPos = rotBack.toRotationMatrix() * newPos;


        Eigen::MatrixXf Rect(2, 4);
        Rect(0, 0) = min_x;
        Rect(1, 0) = min_y;

        Rect(0, 1) = max_x;
        Rect(1, 1) = min_y;

        Rect(0, 2) = max_x;
        Rect(1, 2) = max_y;

        Rect(0, 3) = min_x;
        Rect(1, 3) = max_y;

        Rect = rotBack.toRotationMatrix() * Rect;

        m_rectangle[0] = cv::Point2f(Rect(0, 0), Rect(1, 0));
        m_rectangle[1] = cv::Point2f(Rect(0, 1), Rect(1, 1));
        m_rectangle[2] = cv::Point2f(Rect(0, 2), Rect(1, 2));
        m_rectangle[3] = cv::Point2f(Rect(0, 3), Rect(1, 3));

        m_current_mean[0] = (Rect(0, 0) + Rect(0, 1) + Rect(0, 2) + Rect(0, 3)) / 4.0f;
        m_current_mean[1] = (Rect(1, 0) + Rect(1, 1) + Rect(1, 2) + Rect(1, 3)) / 4.0f;


        m_movement_vector[1] = std::sin(m_state[2]) * 2;
        m_movement_vector[0] = std::cos(m_state[2]) * 2;


        m_movement_vector+= m_current_mean;

        float dx = newPos[0] - m_state[0] + movement_x;
        float dy = newPos[1] - m_state[1] + movement_y;
        if (m_initial_id == 93) {
            std::cout << "Dx: " << dx << std::endl;
            std::cout << "Dy: " << dy << std::endl;
            std::cout << "movement_x: " << movement_x << std::endl;
            std::cout << "movement_y: " << movement_y << std::endl;
            std::cout << "Theta: " << m_state[2] / M_PI * 180 << std::endl;
        }


        float movement = std::sqrt((m_state[0] - newPos[0]) * (m_state[0] - newPos[0]) + (m_state[1] - newPos[1]) * (m_state[1] - newPos[1]));
        //movement =0;
        if (movement > 2) {
            m_state[0] = newPos[0];
            m_state[1] = newPos[1];


            m_state[2] = std::atan2(dy, dx);
        } else {
            m_state[0] += movement_x;
            m_state[1] += movement_y;
        }

        clusterCandidates.clear();
    }

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
    double speed = (std::sqrt((new_state[0] - old_state[0]) * (new_state[0] - old_state[0]) + (new_state[1] - old_state[1]) * (new_state[1] - old_state[1])) * speed_sign) / dt;

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
            (cluster.m_center[0] - m_state[0]) * (cluster.m_center[0] - m_state[0]) + (cluster.m_center[1] - m_state[1]) * (cluster.m_center[1] - m_state[1]));
}

double LidarObstacle::getBoxLongSite() const {
    return m_boxLongSite;
}

void LidarObstacle::setBoxLongSite(double boxLongSite) {
    LidarObstacle::m_boxLongSite = boxLongSite;
}

double LidarObstacle::getBoxShortSite() const {
    return m_boxShortSite;
}

void LidarObstacle::setBoxShortSite(double boxShortSite) {
    LidarObstacle::m_boxShortSite = boxShortSite;
}

double LidarObstacle::getOldBoxLongSite() const {
    return oldBoxLongSite;
}

void LidarObstacle::setOldBoxLongSite(double oldBoxLongSite) {
    LidarObstacle::oldBoxLongSite = oldBoxLongSite;
}

double LidarObstacle::getOldBoxShortSite() const {
    return oldBoxShortSite;
}

void LidarObstacle::setOldBoxShortSite(double oldBoxShortSite) {
    LidarObstacle::oldBoxShortSite = oldBoxShortSite;
}

