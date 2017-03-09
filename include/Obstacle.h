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


public:
    bool isInRect(Point &point);


    int32_t m_confidence = 0;
    double m_rectRot=0;
    uint32_t image_counter = 0;
    double m_speed_x = 10000;
    double m_speed_y = 10000;
    float m_best_width = 0;
    float m_best_length = 0;

    Eigen::Vector2f m_movement_vector;
    Eigen::Vector2f m_current_mean;
    Eigen::Matrix<double, 3, 1> m_state;
    Eigen::Vector2f m_rectangle[4];
    uint64_t m_initial_id;
    std::list<Cluster *> clusterCandidates;


    LidarObstacle(Cluster *cluster, odcore::data::TimeStamp current_time);
    void refresh(double movement_x, double movement_y, odcore::data::TimeStamp current_time, int img_count);
    double getDistance(Cluster &cluster);

};