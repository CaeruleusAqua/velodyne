#define VIS

#include "PointcloudClustering.h"
#include <chrono>

#include "opendlv/scenario/SCNXArchive.h"
#include "opendlv/scenario/SCNXArchiveFactory.h"
#include "opendlv/scenario/ScenarioFactory.h"
#include "opendlv/core/wrapper/graph/DirectedGraph.h"
#include "opendlv/core/wrapper/graph/Edge.h"
#include "opendlv/core/wrapper/graph/Vertex.h"
#include "opendlv/data/environment/EgoState.h"
#include "opendlv/data/environment/Polygon.h"
#include "opendlv/data/environment/Obstacle.h"
#include "opendlv/data/graph/WaypointsEdge.h"
#include "opendlv/data/graph/WaypointVertex.h"
#include "opendlv/data/planning/Route.h"

#include "opendlv/scenario/LaneVisitor.h"
#include "odvdapplanix/GeneratedHeaders_ODVDApplanix.h"
#include <opendavinci/odcore/io/tcp/TCPConnection.h>
#include <opendavinci/odcore/io/tcp/TCPFactory.h>


using namespace std;

// We add some of OpenDaVINCI's namespaces for the sake of readability.
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore::base;
using namespace odcore::data;
using namespace odcore::data;
using namespace automotive;
using namespace automotive::miniature;
using namespace opendlv::data::environment;


PointcloudClustering::PointcloudClustering(const int32_t &argc, char **argv) :
        DataTriggeredConferenceClientModule(argc, argv, "PointcloudClustering"),
        m_old_clusters(), m_obstacles(), gen(rd()) {};

PointcloudClustering::~PointcloudClustering() {}

void PointcloudClustering::setUp() {

    cout << "This method is called before the component's body is executed." << endl;
    cv::namedWindow("Lidar", cv::WINDOW_AUTOSIZE);
    //const odcore::io::URL urlOfSCNXFile(getKeyValueConfiguration().getValue<string>("global.scenario"));
    //core::wrapper::graph::DirectedGraph m_graph;
    //opendlv::scenario::SCNXArchive &scnxArchive = opendlv::scenario::SCNXArchiveFactory::getInstance().getSCNXArchive(
    //        urlOfSCNXFile);

    //m_scenario = &scnxArchive.getScenario();

    // Construct road network.
    //opendlv::scenario::LaneVisitor lv(m_graph, *m_scenario);
    //m_scenario->accept(lv);

    //opendlv::data::scenario::Vertex3 origin = m_scenario->getHeader().getWGS84CoordinateSystem().getOrigin();
    //cout << endl;
    //cout << "Origin: \n" << origin;
    //m_origin = new opendlv::data::environment::WGS84Coordinate(origin.getX(), origin.getY());
    m_origin = new opendlv::data::environment::WGS84Coordinate(57.77284, 12.769964);
    // We are using OpenDaVINCI's std::shared_ptr to automatically
    // release any acquired resources.

    const string RECEIVER = "127.0.0.1";
    const uint32_t PORT = 1234;
    try {
        connection = std::shared_ptr<odcore::io::tcp::TCPConnection>(odcore::io::tcp::TCPFactory::createTCPConnectionTo(RECEIVER, PORT));
    }
    catch (string &exception) {
        cerr << "TCP-connection error: " << exception << endl;
    }

}

void PointcloudClustering::tearDown() {
    cout << "This method is called after the program flow returns from the component's body." << endl;
}

std::list<Point *> PointcloudClustering::getAllPointsNextToSlow(Eigen::Vector2d x, double delta) {
    std::list<Point *> points;
    for (uint32_t i = 0; i < m_cloudSize; i++) {
        for (uint32_t offset = 0; offset < 16; offset++) {
            if (m_points[i][offset].get2Distance(x[0], x[1]) < delta && !m_points[i][offset].isGround()) {
                points.push_back(&m_points[i][offset]);
            }
        }
    }
    return points;
}


void PointcloudClustering::transform(CompactPointCloud &cpc) {
    static const int maping[] = {-15, -13, -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11, 13, 15};
    //static const int maping[] = {-15, -14,-13,-12, -11,-10, -9, -9,-7,-6, -5,-4, -3,-2, -1,0};//, 1,2, 3,4, 5,6, 7,8, 9,10, 11,12, 13,14, 15};
    std::string distances = cpc.getDistances();
    m_cloudSize = distances.size() / 2 / 16;

    m_startAzimuth = utils::deg2rad(cpc.getStartAzimuth() + m_heading);
    m_endAzimuth = utils::deg2rad(cpc.getEndAzimuth() + m_heading);

    vector<double> azimuth_range = utils::linspace(m_startAzimuth, m_endAzimuth, m_cloudSize);


    const uint16_t *data = reinterpret_cast<const uint16_t *>(distances.c_str());

    for (uint32_t i = 0; i < distances.size() / 2; i += 16) {
        for (uint32_t offset = 0; offset < 16; offset++) {
            float measurement = static_cast<float>(data[i + offset]) / 100.0;
            float azimuth = static_cast<float>(azimuth_range[i / 16]);
            float xy_range = measurement * cos(static_cast<float>(utils::deg2rad(maping[offset])));
            float x = xy_range * sin(azimuth);
            float y = xy_range * cos(azimuth);
            float z = measurement * sin(static_cast<float>(utils::deg2rad(maping[offset])));
            m_points[i / 16][offset] = Point(x, y, z, measurement, azimuth);
            m_points[i / 16][offset].setIndex(i / 16, offset);
            if (measurement <= 2.5) {
                m_points[i / 16][offset].setIsGround(true);
                m_points[i / 16][offset].setClustered(true);
                m_points[i / 16][offset].setVisited(true);
            }

        }
    }

}

void PointcloudClustering::segmentGroundByPlane() {
    // devide measurement in sections
    unsigned int sector_size = m_cloudSize / 30;
    std::vector<Point *> minis;
//    for (int sec = 0; sec < 12; sec += 1) {
//        std::vector<Point *> tmp = utils::minZinSec(sector_size * sec, sector_size * (sec + 1), m_points);
//        minis.insert(minis.end(), tmp.begin(), tmp.end());
//    }


    std::vector<Point *> tmp = utils::minZinSec(sector_size * 0 + sector_size / 2, sector_size * (0 + 1) + sector_size / 2, m_points);
    minis.insert(minis.end(), tmp.begin(), tmp.end());
    tmp = utils::minZinSec(sector_size * 12 + sector_size / 2, sector_size * (12 + 1) + sector_size / 2, m_points);
    minis.insert(minis.end(), tmp.begin(), tmp.end());
    tmp = utils::minZinSec(sector_size * 14 + sector_size / 2, sector_size * (14 + 1) + sector_size / 2, m_points);
    minis.insert(minis.end(), tmp.begin(), tmp.end());
    tmp = utils::minZinSec(sector_size * 28 + sector_size / 2, sector_size * (28 + 1) + sector_size / 2, m_points);
    minis.insert(minis.end(), tmp.begin(), tmp.end());

    // RANSAC
    double besterror = 100000000;

    std::uniform_int_distribution<> dis(0, ((minis.size() - 1)));

    for (int probes = 0; probes < 50; probes++) {
        std::vector<Point *> maybeinliers;
        std::vector<Point *> alsoinliers;
        // select 3 random points
        maybeinliers.push_back(minis[dis(gen)]);
        maybeinliers.push_back(minis[dis(gen)]);
        maybeinliers.push_back(minis[dis(gen)]);

        // Triange fitting
//        int offset = dis(gen);
//        maybeinliers.push_back(minis[offset]);
//        maybeinliers.push_back(minis[offset + ((minis.size() - 1) / 3)]);
//        maybeinliers.push_back(minis[offset + 2 * ((minis.size() - 1) / 3)]);


        Plane maybemodel(maybeinliers);

        for (auto &point : minis) {
            if (std::find(maybeinliers.begin(), maybeinliers.end(), point) == maybeinliers.end()) {
                float distance = std::abs(maybemodel.getDist(point->getVec()));
                if (distance < 0.2) {
                    alsoinliers.push_back(point);
                }
            }
        }
        if (alsoinliers.size() > 10) {
            // this implies that we may have found a good model
            // now test how good it is
            Plane plane;
            double err = plane.fitPlaneFromPoints(alsoinliers);
            if (err < besterror && plane.distance > 1.9 && plane.distance < 2.1) {
                besterror = err;
                m_bestGroundModel = plane;
            }
        }
    }
    cout << "BestErr: " << besterror << endl;
    cout << "Groundplane Distance: " << m_bestGroundModel.distance << endl << "Vector: " << endl
         << m_bestGroundModel.normal << endl;

    for (uint32_t i = 0; i < m_cloudSize; i += 1) {
        for (uint32_t offset = 0; offset < 16; offset++) {
            //Eigen::Vector3f point = m_points[i][offset].getVec();
            if (m_bestGroundModel.getDist(m_points[i][offset].getVec()) > -0.3) {
                m_points[i][offset].setVisited(true);
                m_points[i][offset].setClustered(true);
                m_points[i][offset].setIsGround(true);
            }
        }
    }

}


void PointcloudClustering::segmentGroundByHeight() {
    for (uint32_t i = 0; i < m_cloudSize; i += 1) {
        for (uint32_t offset = 0; offset < 16; offset++) {
            //Eigen::Vector3f point = m_points[i][offset].getVec();
            if (m_points[i][offset].getZ() < -1.6) {
                //cout<<"Delete"<<endl;
                m_points[i][offset].setVisited(true);
                m_points[i][offset].setClustered(true);
                m_points[i][offset].setIsGround(true);
            }
        }
    }

}


//void PointcloudClustering::trackObstacles(std::vector<Cluster> &clusters) {
//
//    if (m_old_clusters.size() == 0) {
//        m_old_clusters.insert(m_old_clusters.begin(), clusters.begin(), clusters.end());
//    } else {
//        // correcting clusters with ego movement
//        for (auto &old_cluster : m_old_clusters) {
//            old_cluster.m_center[0] += m_movement_x;
//            old_cluster.m_center[1] += m_movement_y;
//        }
//
//        cout << "Searching Cluster.." << endl;
//        for (auto &new_cluster : clusters) {
//            double min_dist = 10000;
//            Cluster *next = nullptr;
//            for (auto &old_cluster : m_old_clusters) {
//                if (!old_cluster.matched) {
//                    double dist = new_cluster.get2Distance(old_cluster);
//                    if (dist < min_dist) {
//                        next = &old_cluster;
//                        min_dist = dist;
//                    }
//                }
//            }
//            if (next != nullptr && min_dist < 3) {
//                new_cluster.m_id = next->m_id;
//                next->matched = false;
//            }
//        }
//        m_old_clusters.clear();
//        m_old_clusters.insert(m_old_clusters.begin(), clusters.begin(), clusters.end());
//    }
//
//
//    if (m_obstacles.size() == 0) {
//        for (auto &cluster : clusters) {
//            m_obstacles.push_back(LidarObstacle(&cluster, m_current_timestamp));
//        }
//    }
//
//    for (auto &cluster : clusters) {
//
//        unsigned int current_id = cluster.m_id;
//        LidarObstacle *tmp = nullptr;
//        for (auto &obst : m_obstacles) {
//            if (obst.m_initial_id == current_id) {
//                tmp = &obst;
//                obst.clusterCandidates.push_back(&cluster);
//            }
//        }
//        if (tmp == nullptr) {
//            m_obstacles.push_back(LidarObstacle(&cluster, m_current_timestamp));
//        }
//    }
//
//    for (auto &obst : m_obstacles) {
//        obst.refresh(m_movement_x, m_movement_y, m_current_timestamp, m_itCount);
//    }
//
//    std::cout<<"------------------------------------"<<m_obstacles.size()<<endl;
//    m_obstacles.remove_if([] (LidarObstacle &i) {
//        return i.confidenceIsZero();
//    });
//
//
//
//    std::cout<<"------------------------------------"<<m_obstacles.size()<<endl;
//
//}

void PointcloudClustering::trackObstacles(std::vector<Cluster> &clusters) {

    for (auto &cluster : clusters) {
        //cluster.calcRectangle();
        cluster.mean();
    }

    for (auto &obst : m_obstacles) {
        obst.m_filter.predict(obst.getDt(m_current_timestamp));
        double x = obst.m_filter.m_x[0] + m_movement_x;
        double y = obst.m_filter.m_x[1] + m_movement_y;
        for (auto &cluster : clusters) {
            if (!cluster.assigned && cluster.get2Distance(x, y) < 3) {
                cluster.assigned = true;
                obst.clusterCandidates.push_back(&cluster);
            }
        }
        obst.refresh(m_movement_x, m_movement_y, m_current_timestamp, m_itCount);
    }
    for (auto &cluster : clusters) {
        if (!cluster.assigned) {

            m_obstacles.push_back(LidarObstacle(&cluster, m_current_timestamp, m_id_counter++));
            cluster.assigned = true;
        }
    }

    std::cout << "------------------------------------" << m_obstacles.size() << endl;
    m_obstacles.remove_if([](LidarObstacle &i) {
        return i.confidenceIsZero();
    });


    std::cout << "------------------------------------" << m_obstacles.size() << endl;


}


void PointcloudClustering::nextContainer(Container &c) {
    if (c.getDataType() == opendlv::core::sensors::applanix::Grp1Data::ID()) {
        opendlv::core::sensors::applanix::Grp1Data imu = c.getData<opendlv::core::sensors::applanix::Grp1Data>();
        //cout << imu.getHeading() << endl;
        m_lat = imu.getLat();
        m_lon = imu.getLon();
        m_heading = imu.getHeading();


        Point3 cart = m_origin->transform(opendlv::data::environment::WGS84Coordinate(imu.getLat(), imu.getLon()));


        if (imu.getRoll() > 1233) {
            m_x = imu.getLat();
            m_y = imu.getLon();
        } else {
            m_x = cart.getX();
            m_y = cart.getY();
        }


        //std::cout<<"X: "<<m_x<<std::endl;
        //std::cout<<"Y: "<<m_y<<std::endl;

        if (!m_imu_updateted) {
            m_old_x = m_x;
            m_old_y = m_y;
            m_imu_updateted = true;
        }





        //cout << "CartesianPos: " << cart << endl;
        //cv::Mat image(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
        //cv::circle(image, cv::Point(cart.getX() + 500, (-1 * cart.getY()) + 500), 2, cv::Scalar(0, 0, 255));

    }

    if (c.getDataType() == CompactPointCloud::ID()) {
        cout << "-----------------------------" << endl;
        cout << "-----RUN: " << m_itCount << endl;

        m_movement_x = m_x - m_old_x;
        m_movement_y = m_y - m_old_y;
        std::cout << "m_movement_x: " << m_movement_x << endl;
        std::cout << "m_movement_y: " << m_movement_y << endl;

        m_old_x = m_x;
        m_old_y = m_y;


        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        m_current_timestamp = c.getSentTimeStamp();
        CompactPointCloud cpc = c.getData<CompactPointCloud>();
        transform(cpc);
        segmentGroundByHeight();
        //segmentGroundByPlane();


        DbScan dbScan = DbScan(m_points, m_cloudSize);

        std::vector<Cluster> clusters;
        dbScan.getClusters(clusters);

        trackObstacles(clusters);

#ifdef VIS

        const static int res = 1000;
        const static int zoom = 8;

        cv::Mat image(res, res, CV_8UC3, cv::Scalar(0, 0, 0));


        for (uint32_t i = 0; i < m_cloudSize; i++) {
            for (int j = 0; j < 16; j++) {
                Point *point = &m_points[i][j];
                int x = static_cast<int>(point->getX() * zoom) + res / 2;
                int y = -static_cast<int>(point->getY() * zoom) + res / 2;
                if ((x < res) && (y < res) && (y >= 0) && (x >= 0)) {
                    if (point->isGround()) {
                        image.at<cv::Vec3b>(y, x)[0] = 0;
                        image.at<cv::Vec3b>(y, x)[1] = 0;
                        image.at<cv::Vec3b>(y, x)[2] = 255;
                    } else {
                        image.at<cv::Vec3b>(y, x)[0] = 255;
                        image.at<cv::Vec3b>(y, x)[1] = 255;
                        image.at<cv::Vec3b>(y, x)[2] = 255;

                    }
                }

            }
        }





        if (true) {
            cv::circle(image, cv::Point((m_x - m_old_x) * zoom + res / 2, -(m_y - m_old_y) * zoom + res / 2), 4, cv::Scalar(128, 255, 128), 2, 8, 0);
            for (auto &obst : m_obstacles) {
                if (true || obst.m_initial_id == 54) {
                    if (obst.m_confidence >= 2) {

                        if (obst.m_rectangle_center[0] != 0 && obst.m_rectangle_center[1] != 0) {

                            ///// rectangles

                            if (obst.m_best_type == 0)
                                for (int j = 0; j < 4; j++)
                                    cv::line(image, cv::Point(obst.m_rectangle[j][0] * zoom + res / 2, -obst.m_rectangle[j][1] * zoom + res / 2),
                                             cv::Point(obst.m_rectangle[(j + 1) % 4][0] * zoom + res / 2, -obst.m_rectangle[(j + 1) % 4][1] * zoom + res / 2),
                                             cv::Scalar(255, 255, 255), 1, 8);

                            else if (obst.m_best_type == 1)
                                for (int j = 0; j < 4; j++)
                                    cv::line(image, cv::Point(obst.m_rectangle[j][0] * zoom + res / 2, -obst.m_rectangle[j][1] * zoom + res / 2),
                                             cv::Point(obst.m_rectangle[(j + 1) % 4][0] * zoom + res / 2, -obst.m_rectangle[(j + 1) % 4][1] * zoom + res / 2),
                                             cv::Scalar(255, 0, 255),
                                             1, 8);
                            else if (obst.m_best_type == 2)
                                for (int j = 0; j < 4; j++)
                                    cv::line(image, cv::Point(obst.m_rectangle[j][0] * zoom + res / 2, -obst.m_rectangle[j][1] * zoom + res / 2),
                                             cv::Point(obst.m_rectangle[(j + 1) % 4][0] * zoom + res / 2, -obst.m_rectangle[(j + 1) % 4][1] * zoom + res / 2),
                                             cv::Scalar(0, 255, 255),
                                             1, 8);
                            else if (obst.m_best_type == 3)
                                for (int j = 0; j < 4; j++)
                                    cv::line(image, cv::Point(obst.m_rectangle[j][0] * zoom + res / 2, -obst.m_rectangle[j][1] * zoom + res / 2),
                                             cv::Point(obst.m_rectangle[(j + 1) % 4][0] * zoom + res / 2, -obst.m_rectangle[(j + 1) % 4][1] * zoom + res / 2),
                                             cv::Scalar(255, 255, 0),
                                             1, 8);

                            ///////// movement vectors

                            cv::arrowedLine(image, cv::Point(obst.m_filter.m_x[0] * zoom + res / 2, -obst.m_filter.m_x[1] * zoom + res / 2),
                                            cv::Point(obst.m_movement_vector_filtered[0] * zoom + res / 2, -obst.m_movement_vector_filtered[1] * zoom + res / 2),
                                            cv::Scalar(0, 0, 255), 1, 8, 0, 0.1);

                            cv::arrowedLine(image, cv::Point(obst.m_mean_x * zoom + res / 2, -obst.m_mean_y * zoom + res / 2),
                                            cv::Point(obst.m_movement_vector[0] * zoom + res / 2, -obst.m_movement_vector[1] * zoom + res / 2),
                                            cv::Scalar(255, 0, 255), 1, 8, 0, 0.1);

                        }

                        ///////// positions

                        cv::circle(image, cv::Point(obst.m_filter.m_x[0] * zoom + res / 2, -obst.m_filter.m_x[1] * zoom + res / 2), 4, cv::Scalar(0, 0, 255), 2, 8, 0);

                        cv::circle(image, cv::Point(obst.m_state[0] * zoom + res / 2, -obst.m_state[1] * zoom + res / 2), 4, cv::Scalar(255, 0, 255), 2, 8, 0);

                        std::stringstream ss;
                        ss << obst.m_initial_id;
                        cv::putText(image, ss.str(),
                                    cv::Point(obst.m_state[0] * zoom + res / 2, -obst.m_state[1] * zoom + res / 2), cv::FONT_HERSHEY_SIMPLEX, 0.33, cv::Scalar(255, 255, 0));

                    }
                }

            }
            //m_obstacles.clear();

            for (auto &cluster : clusters) {
//            auto hull = utils::convex_hull(cluster);
                //cv::circle(image, cv::Point(cluster.m_center[0] * zoom + res / 2, cluster.m_center[1] * zoom + res / 2), 4, cv::Scalar(255, 0, 255), 2, 8, 0);
//
//            std::vector<cv::Point2f> vec;
//            for (auto &point : hull) {
//                vec.push_back(cv::Point2f(point->getX(), point->getY()));
//            }
//            if (vec.size() > 2) {
//                auto rect = cv::minAreaRect(vec);
//                cv::Point2f rec[4];
//                rect.points(rec);
//                //cv::rectangle(image, rect, cv::Scalar(255, 0, 255), 1, 8, 0 );
//                for (int j = 0; j < 4; j++)
//                    cv::line(image, cv::Point(rec[j].x * zoom + res / 2, rec[j].y * zoom + res / 2),
//                             cv::Point(rec[(j + 1) % 4].x * zoom + res / 2, rec[(j + 1) % 4].y * zoom + res / 2), cv::Scalar(255, 0, 255), 1, 8);
//            }


//            for (uint32_t i = 1; i < hull.size(); i++) {
//                cv::line(image, cv::Point(hull[i - 1]->getX() * zoom + res / 2, hull[i - 1]->getY() * zoom + res / 2),
//                         cv::Point(hull[i]->getX() * zoom + res / 2, hull[i]->getY() * zoom + res / 2),
//                         cv::Scalar(255, 255, 0), 1, 8, 0);
//            }
//            if (hull.size() > 1)
//                cv::line(image, cv::Point(hull[hull.size() - 1]->getX() * zoom + res / 2,
//                                          hull[hull.size() - 1]->getY() * zoom + res / 2),
//                         cv::Point(hull[0]->getX() * zoom + res / 2, hull[0]->getY() * zoom + res / 2),
//                         cv::Scalar(255, 255, 0), 1, 8, 0);
            }

        }


        m_milliseconds += 100;
        m_seconds += m_milliseconds / 1000;
        m_milliseconds = m_milliseconds % 1000;
        m_minutes += m_seconds / 60;
        m_seconds = m_seconds % 60;
        {
            std::stringstream ss;
            ss << "Stamp: " << m_minutes << ":" << m_seconds << ":" << m_milliseconds;
            cv::putText(image, ss.str(),
                        cv::Point(20, res - 20), cv::FONT_HERSHEY_SIMPLEX, 0.33, cv::Scalar(255, 255, 0));

        }
        cv::line(image, cv::Point(res - 20 - zoom, res - 20), cv::Point(res - 20, res - 20), cv::Scalar(255, 255, 0), 1);

        cv::arrowedLine(image, cv::Point(res / 2, res / 2), cv::Point(res / 2 + 50, res / 2),
                        cv::Scalar(255, 255, 255), 1, 8, 0, 0.1);

        cv::arrowedLine(image, cv::Point(res / 2, res / 2), cv::Point(res / 2, res / 2 - 50),
                        cv::Scalar(255, 255, 255), 1, 8, 0, 0.1);


        cv::arrowedLine(image, cv::Point(res / 2, res / 2), cv::Point(m_movement_x * 10 * zoom + res / 2, -m_movement_y * 10 * zoom + res / 2),
                        cv::Scalar(255, 0, 255), 1, 8, 0, 0.1);


        cv::imshow("Lidar", image);
        stringstream ss;




        ss << "../images/img" << m_itCount++ << ".png";
        cv::imwrite(ss.str(), image);


//        for (auto &obst : m_obstacles) {
//            opendlv::core::sensors::applanix::obstacles odvd_obst;
//            odvd_obst.setObjId(obst.m_initial_id);
//            odvd_obst.setSpeed(obst.m_filter.m_x[3]);
//            odvd_obst.setTheta(obst.m_filter.m_x[2]);
//            odvd_obst.setYaw_rate(obst.m_filter.m_x[4]);
//            odvd_obst.setPos_x(obst.m_filter.m_x[0]);
//            odvd_obst.setPos_y(obst.m_filter.m_x[1]);
//            odvd_obst.setType(obst.m_best_type);
//
//            Container c(odvd_obst);
//            getConference().send(c);
//            sleep(100);
//            break;
//        }


        cv::waitKey(1);
#endif
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        stringstream test;
        for (auto &obst : m_obstacles) {
            if (obst.m_confidence >= 2) {
                opendlv::core::sensors::applanix::obstacles tcp_obst;
                tcp_obst.setPos_x(obst.m_filter.m_x[0]);
                tcp_obst.setPos_y(obst.m_filter.m_x[1]);
                tcp_obst.setTheta(obst.m_filter.m_x[2]);
                tcp_obst.setSpeed(obst.m_filter.m_x[3]);
                tcp_obst.setYaw_rate(obst.m_filter.m_x[4]);
                tcp_obst.setType(obst.m_best_type);
                tcp_obst.setObjId(obst.m_initial_id);
                //cout<<obst.m_initial_id<<endl;
                tcp_obst << test;
                test << "::-::";
            }
        }

        try {
            connection->send("startHere::" + test.str());

        }
        catch (string &exception) {
            cerr << "Data could not be sent: " << exception << endl;
        }

        double millis = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000.0;
        millis += std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1000000.0;


        std::cout << "Time difference = " << millis << std::endl;


    }

}
