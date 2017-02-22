#pragma once

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include "opendavinci/generated/odcore/data/CompactPointCloud.h"
#include "opendlv/data/scenario/Scenario.h"
#include "opendlv/data/environment/WGS84Coordinate.h"
#include <list>
#include "Utils.h"
#include "Point.h"
#include <iostream>
#include <random>
#include <array>
#include "dbscan.h"
#include "Cluster.h"
#include "Obstacle.h"
#include "Plane.h"

class PointcloudClustering : public odcore::base::module::DataTriggeredConferenceClientModule {
private:
    /**
     * "Forbidden" copy constructor. Goal: The compiler should warn
     * already at compile time for unwanted bugs caused by any misuse
     * of the copy constructor.
     *
     * @param obj Reference to an object of this class.
     */
    PointcloudClustering(const PointcloudClustering &/*obj*/);

    /**
     * "Forbidden" assignment operator. Goal: The compiler should warn
     * already at compile time for unwanted bugs caused by any misuse
     * of the assignment operator.
     *
     * @param obj Reference to an object of this class.
     * @return Reference to this instance.
     */
    PointcloudClustering &operator=(const PointcloudClustering &/*obj*/);

public:
    /**
     * Constructor.
     *
     * @param argc Number of command line arguments.
     * @param argv Command line arguments.
     */
    PointcloudClustering(const int32_t &argc, char **argv);

    virtual ~PointcloudClustering();

    virtual void nextContainer(odcore::data::Container &c);

private:
    virtual void setUp();

    virtual void tearDown();

    void transform(odcore::data::CompactPointCloud &cpc);
    void segmentGroundByPlane();
    void segmentGroundByHeight();
    void trackObstacles(std::vector<Cluster> &clusters);

    Point m_points[2000][16];
    unsigned int m_cloudSize;
    std::vector<Cluster> m_old_clusters;
    std::list<LidarObstacle> m_obstacles;

    opendlv::data::scenario::Scenario *m_scenario;
    opendlv::data::environment::WGS84Coordinate *m_origin;

    double m_x, m_y, m_lon, m_lat, m_heading=0;
    Plane m_bestGroundModel;


    std::random_device rd;
    std::mt19937 gen;

//    opendlv::core::sensors::applanix::Grp1Data *m_imu;


    Plane m_ground;
    static constexpr float m_eps = 1000;
    static constexpr uint32_t m_minPts = 20;
    unsigned int m_id_counter = 0;

};
