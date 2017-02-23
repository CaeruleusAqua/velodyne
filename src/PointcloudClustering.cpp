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
    const odcore::io::URL urlOfSCNXFile(getKeyValueConfiguration().getValue<string>("global.scenario"));
    core::wrapper::graph::DirectedGraph m_graph;
    opendlv::scenario::SCNXArchive &scnxArchive = opendlv::scenario::SCNXArchiveFactory::getInstance().getSCNXArchive(
            urlOfSCNXFile);

    m_scenario = &scnxArchive.getScenario();

    // Construct road network.
    opendlv::scenario::LaneVisitor lv(m_graph, *m_scenario);
    m_scenario->accept(lv);

    opendlv::data::scenario::Vertex3 origin = m_scenario->getHeader().getWGS84CoordinateSystem().getOrigin();

    cout << "Origin: " << origin;
    m_origin = new opendlv::data::environment::WGS84Coordinate(origin.getX(), origin.getY());

}

void PointcloudClustering::tearDown() {
    cout << "This method is called after the program flow returns from the component's body." << endl;
}


std::list<Point *> PointcloudClustering::getAllPointsNextTo(Eigen::Vector2d x, double delta) {
    double phi = std::atan2(x[1], x[0]);
    double r = x.norm() - delta / 2.0;
    int32_t index = 0;
    double deltaPhi = std::abs((m_endAzimuth - m_startAzimuth) / static_cast<double>(m_cloudSize));
    if (m_endAzimuth < m_startAzimuth) {
        index = (m_startAzimuth - phi) / deltaPhi;

    } else {
        index = (deltaPhi - m_startAzimuth) / deltaPhi;
    }

    double hyp = std::sqrt((delta / 2.0) * (delta / 2.0) + r * r);
    int bound = std::acos(r / hyp) / deltaPhi;

    std::list<Point *> points;
    for (int i = index - bound; i < index + bound; i++) {
        int i_mod = i;
        while (i_mod < 0) {
            i_mod += m_cloudSize;
        }
        while (i_mod > static_cast<int32_t>(m_cloudSize) - 1) {
            i_mod -= m_cloudSize;
        }
        for (uint32_t offset = 0; offset < 16; offset++) {
            if (m_points[i_mod][offset].get2Distance(x[0], x[1]) < delta && !m_points[i_mod][offset].isGround()) {
                points.push_back(&m_points[i_mod][offset]);
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

    m_startAzimuth = utils::deg2rad(-cpc.getStartAzimuth() - m_heading);
    m_endAzimuth = utils::deg2rad(-cpc.getEndAzimuth() - m_heading);


    if (m_startAzimuth <= m_endAzimuth) {
        if (m_startAzimuth < 0) {
            int iter = -m_startAzimuth / (M_PI * 2) + 1;
            if (m_startAzimuth == -M_PI * 2) {
                iter--;
            }
            m_startAzimuth += M_PI * 2 * iter;
            m_endAzimuth += M_PI * 2 * iter;
        }
    } else {
        if (m_endAzimuth < 0) {
            int iter = -m_endAzimuth / (M_PI * 2) + 1;
            if (m_endAzimuth == -M_PI * 2) {
                iter--;
            }
            std::cout << "iter: " << iter << endl;
            m_startAzimuth += M_PI * 2 * iter;
            m_endAzimuth += M_PI * 2 * iter;
        }

    }

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
            if (measurement <= 1.8)
                m_points[i / 16][offset].setIsGround(true);


        }
    }

}

void PointcloudClustering::segmentGroundByPlane() {
    // devide measurement in sections
    unsigned int sector_size = m_cloudSize / 8;
    std::vector<Point *> minis;
    for (int sec = 0; sec < 8; sec += 1) {
        std::vector<Point *> tmp = utils::minZinSec(sector_size * sec, sector_size * (sec + 1), m_points);
        minis.insert(minis.end(), tmp.begin(), tmp.end());
    }


//    std::vector<Point *> tmp = utils::minZinSec(sector_size * 0, sector_size * (0 + 1), m_points);
//    minis.insert(minis.end(), tmp.begin(), tmp.end());
//    tmp = utils::minZinSec(sector_size * 11, sector_size * (11 + 1), m_points);
//    minis.insert(minis.end(), tmp.begin(), tmp.end());
//    tmp = utils::minZinSec(sector_size * 5, sector_size * (5 + 1), m_points);
//    minis.insert(minis.end(), tmp.begin(), tmp.end());
//    tmp = utils::minZinSec(sector_size * 6, sector_size * (6 + 1), m_points);
//    minis.insert(minis.end(), tmp.begin(), tmp.end());

    // RANSAC
    double besterror = 100000000;

    std::uniform_int_distribution<> dis(0, ((minis.size() - 1) / 3));

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
            if (m_bestGroundModel.getDist(m_points[i][offset].getVec()) > -0.5) {
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
            if (m_points[i][offset].getZ() < -1.5) {
                //cout<<"Delete"<<endl;
                m_points[i][offset].setVisited(true);
                m_points[i][offset].setClustered(true);
                m_points[i][offset].setIsGround(true);
            }
        }
    }

}


void PointcloudClustering::trackObstacles(std::vector<Cluster> &clusters) {


    if (m_old_clusters.size() == 0) {
        m_old_clusters.insert(m_old_clusters.begin(), clusters.begin(), clusters.end());
    } else {
        cout << "Searching Cluster.." << endl;
        for (auto &new_cluster : clusters) {
            double min_dist = 10000;
            Cluster *next = nullptr;
            for (auto &old_cluster : m_old_clusters) {
                if (!old_cluster.matched) {
                    double dist = new_cluster.get2Distance(old_cluster);
                    if (dist < min_dist) {
                        next = &old_cluster;
                        min_dist = dist;
                    }
                }
            }
            if (next != nullptr && min_dist < 3) {
                new_cluster.m_id = next->m_id;
                next->matched = false;
            }
        }
        m_old_clusters.clear();
        m_old_clusters.insert(m_old_clusters.begin(), clusters.begin(), clusters.end());
    }



    // initial run
    //cout << "Tracking Objects!" << endl;
    if (m_obstacles.empty()) {
        cout << "Initial tracking Objects..." << endl;
        for (auto &cluster : clusters) {
            if (cluster.getRectLongSite() < 7 && cluster.getRectShortSite() < 3) {
                //classify Car or pedestrian
                m_obstacles.push_back(LidarObstacle(cluster.m_center[0], cluster.m_center[1], cluster.getTheta(), 0, 0, &cluster, m_current_timestamp, true));
            }


        }
    } else {
        //cout << "Repeat tracking Objects... Clusters to track: " << clusters.size() << endl;
        //cout << "Obstacles to track: " << m_obstacles.size() << endl;
        for (auto &obst : m_obstacles) {
            obst.predict(m_current_timestamp);
        }
        double distThreshold = 3.0;
        LidarObstacle *closest_obstacle = nullptr;
        for (auto &cluster : clusters) {
            double closest_dist = 10000000;
            for (auto &obst : m_obstacles) {
                double dist = obst.getDistance(cluster);
                if (dist < closest_dist && dist < distThreshold) {
                    closest_obstacle = &obst;
                    closest_dist = dist;
                }
            }
            if (closest_obstacle != nullptr) {
                //cout<<"Appending cluster to object!!"<<endl;
                if (cluster.m_id == 93 || cluster.m_id == 95) {
                    cout << "Append Cluster" << cluster.m_id << " with distance: " << closest_dist << " to " << closest_obstacle->m_initial_id << endl;
                }
                closest_obstacle->clusterCandidates.push_back(&cluster);
            }

        }


        std::list<std::_List_iterator<LidarObstacle>> invalid_obstacles;

        for (auto obst = m_obstacles.begin(); obst != m_obstacles.end(); obst++) {
            //cout << "num clusterCandidates: " << obst.clusterCandidates.size() << endl;

            double closest_dist = 10000000;
            Cluster *closest_cluster = nullptr;
            Cluster newCluster = Cluster();
            for (auto cluster : obst->clusterCandidates) {
                double dist = obst->getDistance(*cluster);
                if (dist < closest_dist) {
                    closest_dist = dist;
                    closest_cluster = cluster;
                }
            }


            if (closest_cluster != nullptr) {
                Eigen::Vector2d tmp;

                tmp << obst->m_predicted[0], obst->m_predicted[1];
                if (closest_cluster->getRectLongSite() > obst->m_box_size) {
                    obst->m_box_size = closest_cluster->getRectLongSite();
                }
                std::list<Point *> newPoints = getAllPointsNextTo(tmp, obst->m_box_size);
                for (auto point : newPoints) {
                    closest_cluster->m_cluster.push_back(point);
                }
                closest_cluster->calcRectangle();
                if (closest_cluster->getRectLongSite() > 7 || closest_cluster->getRectShortSite() > 3) {
                    invalid_obstacles.push_back(obst);
                } else {


                    if (closest_cluster->getRectLongSite() < obst->m_box_size) {
                        //wenn lange seite kuerzer als frueher
                        //finde punkt am naechsten zum ursprung
                        double dist0 = cv::norm(closest_cluster->m_rectangle[0]);
                        double dist1 = cv::norm(closest_cluster->m_rectangle[1]);
                        double dist2 = cv::norm(closest_cluster->m_rectangle[2]);
                        double dist3 = cv::norm(closest_cluster->m_rectangle[3]);
                        double dx = 0, dy = 0, rot = 0;
                        if (dist0 <= dist1 && dist0 <= dist2 && dist0 <= dist3) {
                            double lenA = cv::norm(closest_cluster->m_rectangle[0] - closest_cluster->m_rectangle[1]);
                            double lenB = cv::norm(closest_cluster->m_rectangle[0] - closest_cluster->m_rectangle[3]);
                            if (lenA <= lenB) { // kurze seite
                                // make sure taking point with lowest y , i'm unsure that this is enough
                                if (closest_cluster->m_rectangle[0].y < closest_cluster->m_rectangle[1].y) {
                                    dx = closest_cluster->m_rectangle[0].x;
                                    dy = closest_cluster->m_rectangle[0].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[1].y - dy, closest_cluster->m_rectangle[1].x - dx);
                                } else {
                                    dx = closest_cluster->m_rectangle[1].x;
                                    dy = closest_cluster->m_rectangle[1].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[0].y - dy, closest_cluster->m_rectangle[0].x - dx);
                                }
                            } else {
                                // make sure taking point with lowest y , i'm unsure that this is enough
                                if (closest_cluster->m_rectangle[0].y < closest_cluster->m_rectangle[3].y) {
                                    dx = closest_cluster->m_rectangle[0].x;
                                    dy = closest_cluster->m_rectangle[0].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[3].y - dy, closest_cluster->m_rectangle[3].x - dx);
                                } else {
                                    dx = closest_cluster->m_rectangle[3].x;
                                    dy = closest_cluster->m_rectangle[3].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[0].y - dy, closest_cluster->m_rectangle[0].x - dx);
                                }

                            }
                        } else if (dist1 <= dist0 && dist1 <= dist2 && dist1 <= dist3) {
                            double lenA = cv::norm(closest_cluster->m_rectangle[1] - closest_cluster->m_rectangle[0]);
                            double lenB = cv::norm(closest_cluster->m_rectangle[1] - closest_cluster->m_rectangle[2]);
                            if (lenA <= lenB) { // kurze seite
                                // make sure taking point with lowest y , i'm unsure that this is enough
                                if (closest_cluster->m_rectangle[1].y < closest_cluster->m_rectangle[0].y) {
                                    dx = closest_cluster->m_rectangle[1].x;
                                    dy = closest_cluster->m_rectangle[1].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[0].y - dy, closest_cluster->m_rectangle[0].x - dx);
                                } else {
                                    dx = closest_cluster->m_rectangle[0].x;
                                    dy = closest_cluster->m_rectangle[0].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[1].y - dy, closest_cluster->m_rectangle[1].x - dx);
                                }
                            } else {
                                // make sure taking point with lowest y , i'm unsure that this is enough
                                if (closest_cluster->m_rectangle[1].y < closest_cluster->m_rectangle[2].y) {
                                    dx = closest_cluster->m_rectangle[1].x;
                                    dy = closest_cluster->m_rectangle[1].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[2].y - dy, closest_cluster->m_rectangle[2].x - dx);
                                } else {
                                    dx = closest_cluster->m_rectangle[2].x;
                                    dy = closest_cluster->m_rectangle[2].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[1].y - dy, closest_cluster->m_rectangle[1].x - dx);
                                }

                            }
                        } else if (dist2 <= dist0 && dist2 <= dist1 && dist2 <= dist3) {
                            double lenA = cv::norm(closest_cluster->m_rectangle[2] - closest_cluster->m_rectangle[1]);
                            double lenB = cv::norm(closest_cluster->m_rectangle[2] - closest_cluster->m_rectangle[3]);
                            if (lenA <= lenB) { // kurze seite
                                // make sure taking point with lowest y , i'm unsure that this is enough
                                if (closest_cluster->m_rectangle[2].y < closest_cluster->m_rectangle[1].y) {
                                    dx = closest_cluster->m_rectangle[2].x;
                                    dy = closest_cluster->m_rectangle[2].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[1].y - dy, closest_cluster->m_rectangle[1].x - dx);
                                } else {
                                    dx = closest_cluster->m_rectangle[1].x;
                                    dy = closest_cluster->m_rectangle[1].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[2].y - dy, closest_cluster->m_rectangle[2].x - dx);
                                }
                            } else {
                                // make sure taking point with lowest y , i'm unsure that this is enough
                                if (closest_cluster->m_rectangle[2].y < closest_cluster->m_rectangle[3].y) {
                                    dx = closest_cluster->m_rectangle[2].x;
                                    dy = closest_cluster->m_rectangle[2].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[3].y - dy, closest_cluster->m_rectangle[3].x - dx);
                                } else {
                                    dx = closest_cluster->m_rectangle[3].x;
                                    dy = closest_cluster->m_rectangle[3].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[2].y - dy, closest_cluster->m_rectangle[2].x - dx);
                                }

                            }
                        } else if (dist3 <= dist0 && dist3 <= dist1 && dist3 <= dist2) {
                            double lenA = cv::norm(closest_cluster->m_rectangle[3] - closest_cluster->m_rectangle[0]);
                            double lenB = cv::norm(closest_cluster->m_rectangle[3] - closest_cluster->m_rectangle[2]);
                            if (lenA <= lenB) { // kurze seite
                                // make sure taking point with lowest y , i'm unsure that this is enough
                                if (closest_cluster->m_rectangle[3].y < closest_cluster->m_rectangle[0].y) {
                                    dx = closest_cluster->m_rectangle[3].x;
                                    dy = closest_cluster->m_rectangle[3].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[0].y - dy, closest_cluster->m_rectangle[0].x - dx);
                                } else {
                                    dx = closest_cluster->m_rectangle[0].x;
                                    dy = closest_cluster->m_rectangle[0].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[3].y - dy, closest_cluster->m_rectangle[3].x - dx);
                                }
                            } else {
                                // make sure taking point with lowest y , i'm unsure that this is enough
                                if (closest_cluster->m_rectangle[3].y < closest_cluster->m_rectangle[2].y) {
                                    dx = closest_cluster->m_rectangle[3].x;
                                    dy = closest_cluster->m_rectangle[3].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[2].y - dy, closest_cluster->m_rectangle[2].x - dx);
                                } else {
                                    dx = closest_cluster->m_rectangle[2].x;
                                    dy = closest_cluster->m_rectangle[2].y;
                                    rot = std::atan2(closest_cluster->m_rectangle[3].y - dy, closest_cluster->m_rectangle[3].x - dx);
                                }

                            }

                        }
                        //construct new rectangle
                        Eigen::Rotation2D<double> rotma(rot);
                        Eigen::Vector2d x1(0, 0);
                        Eigen::Vector2d x2(closest_cluster->getRectShortSite(), 0);
                        Eigen::Vector2d x3(closest_cluster->getRectShortSite(), obst->m_box_size);
                        Eigen::Vector2d x4(0, obst->m_box_size);

                        x1 = rotma.toRotationMatrix() * x1;
                        x2 = rotma.toRotationMatrix() * x2;
                        x3 = rotma.toRotationMatrix() * x3;
                        x4 = rotma.toRotationMatrix() * x4;

                        closest_cluster->m_rectangle[0] = cv::Point2f(x1[0] + dx, x1[1] + dy);
                        closest_cluster->m_rectangle[1] = cv::Point2f(x2[0] + dx, x2[1] + dy);
                        closest_cluster->m_rectangle[2] = cv::Point2f(x3[0] + dx, x3[1] + dy);
                        closest_cluster->m_rectangle[3] = cv::Point2f(x4[0] + dx, x4[1] + dy);


                    }


                    closest_cluster->meanRect();

                    obst->update(closest_cluster->m_center[0], closest_cluster->m_center[1], -closest_cluster->getTheta(), m_current_timestamp);


                    obst->m_rectangle[0] = cv::Point2f(closest_cluster->m_rectangle[0]);
                    obst->m_rectangle[1] = cv::Point2f(closest_cluster->m_rectangle[1]);
                    obst->m_rectangle[2] = cv::Point2f(closest_cluster->m_rectangle[2]);
                    obst->m_rectangle[3] = cv::Point2f(closest_cluster->m_rectangle[3]);
                }

            }

            if (obst->clusterCandidates.size() == 0) {
                obst->lostTrackingCounts++;
            } else {
                obst->lostTrackingCounts--;
            }
            if (obst->lostTrackingCounts > 20) {
                invalid_obstacles.push_back(obst);
            }
            obst->clusterCandidates.clear();
            if (obst->m_initial_id == 93) {
                cout << "Roatation: " << obst->m_state[2] / M_PI * 180 << endl;
                cout << "Speed: " << obst->m_state[3] << endl;
                cout << "Yawrate: " << obst->m_state[4] / M_PI * 180 << endl;
                cout << "Rect: :" << obst->m_rectangle[0] << " : " << obst->m_rectangle[1] << " : " << obst->m_rectangle[2] << " : " << obst->m_rectangle[3] << endl;


            }
        }
        for (auto &remObst : invalid_obstacles) {
            m_obstacles.erase(remObst);
        }


    }


}

void PointcloudClustering::nextContainer(Container &c) {
    if (c.getDataType() == opendlv::core::sensors::applanix::Grp1Data::ID()) {
        opendlv::core::sensors::applanix::Grp1Data imu = c.getData<opendlv::core::sensors::applanix::Grp1Data>();
        //cout << imu.getHeading() << endl;
        m_lat = imu.getLat();
        m_lon = imu.getLon();
        m_heading = imu.getHeading();

        Point3 cart = m_origin->transform(opendlv::data::environment::WGS84Coordinate(imu.getLat(), imu.getLon()));
        m_x = cart.getX();
        m_y = cart.getY();
        //cout << "CartesianPos: " << cart << endl;
        //cv::Mat image(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
        //cv::circle(image, cv::Point(cart.getX() + 500, (-1 * cart.getY()) + 500), 2, cv::Scalar(0, 0, 255));

    }

    if (c.getDataType() == CompactPointCloud::ID()) {

        cout << "-----------------------------" << endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        m_current_timestamp = c.getSentTimeStamp();
        CompactPointCloud cpc = c.getData<CompactPointCloud>();
        transform(cpc);
        segmentGroundByHeight();


        DbScan dbScan = DbScan(m_points, m_cloudSize);

        std::vector<Cluster> clusters;
        dbScan.getClusters(clusters);
        for (auto &cluster : clusters) {
            cluster.m_id = m_id_counter++;
            cluster.calcRectangle();
            cluster.meanRect();
        }
        trackObstacles(clusters);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();


        double millis = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000.0;
        millis += std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1000000.0;

        std::cout << "Time difference = " << millis << std::endl;
        //cout << "--------------------------" << endl << endl;

        const static int res = 1000;
        const static int zoom = 12;


        cv::Mat image(res, res, CV_8UC3, cv::Scalar(0, 0, 0));


        for (auto &layer : m_scenario->getListOfLayers()) {
            //cout << layer.getLongName() << endl;
            for (auto &road : layer.getListOfRoads()) {
                //cout << road.getLongName() << endl;
                for (auto &lane : road.getListOfLanes()) {
                    opendlv::data::scenario::LaneModel *model = lane.getLaneModel();
                    if (model->getType() == model->POINTMODEL) {
                        opendlv::data::scenario::PointModel *pointmodel = static_cast<opendlv::data::scenario::PointModel *>(model);
                        vector<opendlv::data::scenario::IDVertex3> vertexes = pointmodel->getListOfIdentifiableVertices();
                        bool first = false;
                        cv::Point oldPoint = cv::Point(0, 0);
                        for (auto &vertex : vertexes) {
                            cv::Point newPoint = cv::Point((vertex.getX() - m_x) * zoom + res / 2,
                                                           (-1 * (vertex.getY() - m_y)) * zoom + res / 2);
                            if (first) {
                                cv::line(image, oldPoint,
                                         newPoint, cv::Scalar(255, 255, 0), 1, 8, 0);
                            }
                            oldPoint = newPoint;
                            first = true;
                        }
                    }

                }
            }
        }


        for (uint32_t i = 0; i < m_cloudSize; i++) {
            for (int j = 0; j < 16; j++) {
                Point *point = &m_points[i][j];
                int x = static_cast<int>(point->getX() * zoom) + res / 2;
                int y = static_cast<int>(point->getY() * zoom) + res / 2;
                if ((x < 800) && (y < 800) && (y >= 0) && (x >= 0)) {
                    // if (point->getMeasurement() > 1 && ( point->getPos()[3] <0  && point->getPos()[3] > -1)) {
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


        for (auto &obst : m_obstacles) {

            std::stringstream ss;
            ss << obst.m_initial_id;

            cv::putText(image, ss.str(),
                        cv::Point(obst.m_state[0] * zoom + res / 2, obst.m_state[1] * zoom + res / 2),
                        cv::FONT_HERSHEY_SIMPLEX, 0.33,
                        cv::Scalar(255, 0, 0));


            if (obst.m_initial_id == 93 || obst.m_initial_id == 95) {
                cv::circle(image, cv::Point(obst.m_predicted[0] * zoom + res / 2, obst.m_predicted[1] * zoom + res / 2), 4, cv::Scalar(0, 255, 255), 2, 8, 0);
                cv::circle(image, cv::Point(obst.m_state[0] * zoom + res / 2, obst.m_state[1] * zoom + res / 2), 4, cv::Scalar(255, 0, 255), 2, 8, 0);
            }
            for (int j = 0; j < 4; j++)
                cv::line(image, cv::Point(obst.m_rectangle[j].x * zoom + res / 2, obst.m_rectangle[j].y * zoom + res / 2),
                         cv::Point(obst.m_rectangle[(j + 1) % 4].x * zoom + res / 2, obst.m_rectangle[(j + 1) % 4].y * zoom + res / 2), cv::Scalar(255, 0, 255), 1, 8);
        }
        //m_obstacles.clear();

        for (auto &cluster : clusters) {
//            auto hull = utils::convex_hull(cluster);
            std::stringstream ss;
            ss << cluster.m_id;
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

            cv::putText(image, ss.str(),
                        cv::Point(cluster.m_center[0] * zoom + res / 2, cluster.m_center[1] * zoom + res / 2),
                        cv::FONT_HERSHEY_SIMPLEX, 0.33,
                        cv::Scalar(255, 255, 255));
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


        cv::line(image, cv::Point(res - 20 - zoom, res - 20), cv::Point(res - 20, res - 20), cv::Scalar(255, 255, 0),
                 1);


        cv::imshow("Lidar", image);

        cv::waitKey(1);
    }

}

