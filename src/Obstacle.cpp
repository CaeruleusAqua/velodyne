#include "Obstacle.h"
#include <math.h>
#include <list>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

LidarObstacle::LidarObstacle(Cluster *cluster, odcore::data::TimeStamp current_time) : clusterCandidates() {
    m_latestTimestamp = current_time;
    m_state << cluster->m_center[0], cluster->m_center[1], 0;
    m_initial_id = cluster->m_id;
}


void LidarObstacle::refresh(double movement_x, double movement_y, odcore::data::TimeStamp current_time, int img_count) {
    if (clusterCandidates.size() > 0) {
        Eigen::Rotation2D<float> rot(-m_state[2]);
        std::list<Eigen::Vector2f> points;

        for (auto &cluster : clusterCandidates) {
            for (auto &point : cluster->m_cluster) {
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

        Eigen::MatrixXf Rect(2, 4);
        float thetaCorrection = 0;

//        if(m_initial_id == 93) {
//
//            const static int zoom = 48;
//            int res_x = int((max_x - min_x) * zoom);
//            int res_y = int((max_y - min_y) * zoom);
//            cv::Mat image(res_y, res_x, CV_8UC3, cv::Scalar(0, 0, 0));
//
//            for (auto &point : points) {
//                int x = static_cast<int>((point[0] - min_x) * zoom);
//                int y = res_y - static_cast<int>((point[1] - min_y) * zoom);
//
//                //std::cout << "Size: " << res_x << " : " << res_y << std::endl;
//                //std::cout << "X: " << x << std::endl;
//                //std::cout << "Y: " << y << std::endl;
//                if ((x < res_x) && (y < res_y) && (y >= 0) && (x >= 0)) {
//                    image.at<cv::Vec3b>(y, x)[0] = 255;
//                    image.at<cv::Vec3b>(y, x)[1] = 255;
//                    image.at<cv::Vec3b>(y, x)[2] = 255;
//                }
//            }
//
//            std::stringstream ss;
//            ss << "../images/Obst" << m_initial_id << "_" << img_count++ << ".bmp";
//            cv::imwrite(ss.str(), image);
//        }

        float p1_x, p1_y, p2_x, p2_y;
        if ((min_y + max_y) / 2 > 0) {
            // minimize y if are above the origin

            p1_x = 1000;
            p1_y = 1000;
            p2_x = 1000;
            p2_y = 1000;

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


            thetaCorrection = std::atan2(p2_y - p1_y, p2_x - p1_x);
            Eigen::Rotation2D<float> rotCorrection(-thetaCorrection);
            for (auto &point : points) {
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


            float height = getMostPropWidth(max_y - min_y);
            float width = getMostPropLenght(max_x - min_x);


            Rect(0, 0) = max_x - width;
            Rect(1, 0) = min_y;

            Rect(0, 1) = max_x;
            Rect(1, 1) = min_y;

            Rect(0, 2) = max_x;
            Rect(1, 2) = min_y + height;

            Rect(0, 3) = max_x - width;
            Rect(1, 3) = min_y + height;


        } else {
            // maximize y if are under the origin
            p1_x = -1000;
            p1_y = -1000;
            p2_x = -1000;
            p2_y = -1000;

            for (auto &vec : points) {
                if (vec[0] < x1) {
                    //std::cout<<"------ vec[1]: "<<vec[1]<<std::endl;
                    if (vec[1] > p1_y) {
                        //std::cout<<"------ vec[12]: "<<vec[1]<<std::endl;
                        p1_x = vec[0];
                        p1_y = vec[1];
                    }
                }
                if (vec[0] > x2) {
                    if (vec[1] > p2_y) {
                        p2_x = vec[0];
                        p2_y = vec[1];
                    }

                }
            }


            thetaCorrection = std::atan2(p2_y - p1_y, p2_x - p1_x);

            Eigen::Rotation2D<float> rotCorrection(-thetaCorrection);
            for (auto &point : points) {
                point = rotCorrection.toRotationMatrix() * point;
            }

            if(m_initial_id == 93) {
                std::cout<<"------ thetaCorrection: "<<thetaCorrection<<std::endl;
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


            float height = getMostPropWidth(max_y - min_y);
            float width = getMostPropLenght(max_x - min_x);


            Rect(0, 0) = max_x - width;
            Rect(1, 0) = max_y - height;

            Rect(0, 1) = max_x;
            Rect(1, 1) = max_y - height;

            Rect(0, 2) = max_x;
            Rect(1, 2) = max_y;

            Rect(0, 3) = max_x - width;
            Rect(1, 3) = max_y;


        }

        if(m_initial_id == 93) {
            for(auto &width : m_width){
                std::cout<<width[0]<<";"<<width[1]<<"-";
            }
            std::cout<<std::endl;
        }
        if(m_initial_id == 93) {
            for(auto &length : m_length){
                std::cout<<length[0]<<";"<<length[1]<<"-";
            }
            std::cout<<std::endl;
        }


        Eigen::Vector2f newPos(max_x, (max_y + min_y) / 2);
        Eigen::Rotation2D<float> rotBack(m_state[2] + thetaCorrection);
        m_rectRot = m_state[2] + thetaCorrection;
        newPos = rotBack.toRotationMatrix() * newPos;

        Rect = rotBack.toRotationMatrix() * Rect;


        m_rectangle[0] = Eigen::Vector2f(Rect(0, 0), Rect(1, 0));
        m_rectangle[1] = Eigen::Vector2f(Rect(0, 1), Rect(1, 1));
        m_rectangle[2] = Eigen::Vector2f(Rect(0, 2), Rect(1, 2));
        m_rectangle[3] = Eigen::Vector2f(Rect(0, 3), Rect(1, 3));



        float oldPosX = m_current_mean[0];
        float oldPosY = m_current_mean[1];
        m_current_mean[0] = (Rect(0, 0) + Rect(0, 1) + Rect(0, 2) + Rect(0, 3)) / 4.0f;
        m_current_mean[1] = (Rect(1, 0) + Rect(1, 1) + Rect(1, 2) + Rect(1, 3)) / 4.0f;
        double dt = (current_time - m_latestTimestamp).toMicroseconds() / 1000000.0d;

        double speed = 2;
        if (oldPosX != 0 || oldPosY != 0) {
            m_speed_x = 0.5 * ((m_current_mean[0] - oldPosX) / dt) + m_speed_x * 0.5;
            m_speed_y = 0.5 * ((m_current_mean[1] - oldPosY) / dt) + m_speed_y * 0.5;
            speed = std::sqrt(m_speed_x * m_speed_x + m_speed_y * m_speed_y);
            //std::cout << "Speed: " << speed << std::endl;
        }


        m_movement_vector[1] = std::sin(m_state[2]) * speed;
        m_movement_vector[0] = std::cos(m_state[2]) * speed;


        if(speed> 10)
            m_confidence/=2;


        m_movement_vector += m_current_mean;

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

        m_latestTimestamp = current_time;

        clusterCandidates.clear();
        m_confidence++;
    } else {
        m_confidence /= 2;
    }


}


double LidarObstacle::getDistance(Cluster &cluster) {
    return std::sqrt(
            (cluster.m_center[0] - m_state[0]) * (cluster.m_center[0] - m_state[0]) + (cluster.m_center[1] - m_state[1]) * (cluster.m_center[1] - m_state[1]));
}


bool LidarObstacle::isInRect(Point &point) {
    Eigen::Rotation2D<float> rotCorrection(-m_rectRot);
    Eigen::Vector2f rrect[4];
    for (int i = 0; i < 4; i++) {
        rrect[i] = rotCorrection.toRotationMatrix() * m_rectangle[i];
    }
}




