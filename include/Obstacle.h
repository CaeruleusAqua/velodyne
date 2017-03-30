#pragma once

#include <vector>
#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include "Point.h"
#include "Kalman.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "Cluster.h"
#include <list>

class LidarObstacle {
private:
    odcore::data::TimeStamp m_latestTimestamp;
    std::list<std::array<int,2>> m_width;
    std::list<std::array<int,2>> m_length;


    float getMostPropWidth(float width){
        int mapped = (width*2)+1;
        bool found = false;
        for( auto &pair : m_width){
            if(pair[0]==mapped){
                pair[1]++;
                found = true;
            }
        }
        if(!found){
            std::array<int,2> newSize = {mapped,1};
            m_width.push_back(newSize);
        }
        int max_prop = 0;
        double retval = 0;
        for( auto &pair : m_width){
            if(pair[1]>max_prop){
                max_prop=pair[1];
                retval = pair[0]/2.0;
            }
        }
        return retval;
    }

    float getMostPropLenght(float lenght){
        int mapped = (lenght*2)+1;
        bool found = false;
        for( auto &pair : m_length){
            if(pair[0]==mapped){
                pair[1]++;
                found = true;
            }
        }
        if(!found){
            std::array<int,2> newSize = {mapped,1};
            m_length.push_back(newSize);
        }
        int max_prop = 0;
        double retval = 0;
        for( auto &pair : m_length){
            if(pair[1]>max_prop){
                max_prop=pair[1];
                retval = pair[0]/2.0;
            }
        }
        return retval;
    }


    void updateRectangle();


public:

    int32_t m_confidence = 1;
    uint64_t m_initial_id = 0;
    uint32_t image_counter = 0;

    double m_rectRot=0;
    double m_rectRot_old=0;

    double m_speed_x = 0;
    double m_speed_y = 0;
    float m_best_width = 0;
    float m_best_length = 0;
    // 0 -unclassified ; 1 - Car ; 2 cycelist ; 3 - pedestrian
    int m_best_type=0;
    float m_max_height = 0;
    double m_mean_x=0;
    double m_mean_y=0;
    bool initial_theta = false;


    Eigen::Vector2f m_movement_vector;
    Eigen::Vector2f m_movement_vector_filtered;

    Eigen::Vector2f m_rectangle[4];
    Eigen::Vector2f m_rectangle_center;


    Eigen::Matrix<double, 3, 1> m_state;



    Kalman m_filter;

    std::list<Cluster *> clusterCandidates;

    bool isInRect(Point &point);
    LidarObstacle(Cluster *cluster, odcore::data::TimeStamp current_time, uint64_t id);
    void refresh(double movement_x, double movement_y, odcore::data::TimeStamp current_time, int img_count);
    double getDistance(Cluster &cluster);
    bool confidenceIsZero();
    double getDt(odcore::data::TimeStamp current_time);
};