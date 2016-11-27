#pragma once

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include "opendavinci/generated/odcore/data/CompactPointCloud.h"
#include "opendavinci/odcore/wrapper/half_float.h"
#include "Utils.h"
#include "Point.h"
#include <iostream>

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

    std::vector<std::vector<Point>> transform(odcore::data::CompactPointCloud cpc);

    std::vector<Point*> regionQuery(Point *point, double eps);

    void expandCluster(std::vector<Point*> &neighbors, std::vector<Point*> &cluster);

    std::vector<std::vector<Point>> m_points;
};
