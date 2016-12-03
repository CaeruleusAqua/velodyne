#include "PointcloudClustering.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <chrono>
#include "dbscan.h"
#include "Cluster.h"


using namespace std;

// We add some of OpenDaVINCI's namespaces for the sake of readability.
using namespace odcore::base::module;
using namespace odcore::data;

PointcloudClustering::PointcloudClustering(const int32_t &argc, char **argv) :
        DataTriggeredConferenceClientModule(argc, argv, "PointcloudClustering") {};

PointcloudClustering::~PointcloudClustering() {}

void PointcloudClustering::setUp() {
    cout << "This method is called before the component's body is executed." << endl;
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
}

void PointcloudClustering::tearDown() {
    cout << "This method is called after the program flow returns from the component's body." << endl;
}


void PointcloudClustering::transform(CompactPointCloud &cpc) {
    static const int maping[] = {-15, -13, -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11, 13, 15};
    std::string distances = cpc.getDistances();
    m_cloudSize = distances.size() / 2 / 16;
    vector<double> azimuth_range = utils::linspace(utils::deg2rad(cpc.getStartAzimuth()), utils::deg2rad(cpc.getEndAzimuth()), m_cloudSize);
    const half *data = reinterpret_cast<const half *>(distances.c_str());

    for (uint32_t i = 0; i < distances.size() / 2; i += 16) {
        for (uint32_t offset = 0; offset < 16; offset++) {
            float measurement = static_cast<float>(data[i + offset]);
            float azimuth = static_cast<float>(azimuth_range[i / 16]);
            float xy_range = measurement * cos(static_cast<float>(utils::deg2rad(maping[offset])));
            float x = xy_range * sin(azimuth);
            float y = xy_range * cos(azimuth);
            float z = measurement * sin(static_cast<float>(utils::deg2rad(maping[offset])));
            m_points[i / 16][offset] = Point(x, y, z, measurement, azimuth);
            m_points[i / 16][offset].setIndex(i / 16, offset);
            if (measurement <= 1 || !(z < 10 && z > -1.5)) {
                m_points[i / 16][offset].setVisited(true);


                //// not jet correctly fixed.. ground should not connect clusters...


                m_points[i / 16][offset].setClustered(true);
            }
        }
    }




}


void PointcloudClustering::nextContainer(Container &c) {
    if (c.getDataType() == CompactPointCloud::ID()) {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        CompactPointCloud cpc = c.getData<CompactPointCloud>();

        transform(cpc);


        DbScan dbScan = DbScan(m_points, m_cloudSize);

        std::vector<Cluster> clusters;
        dbScan.getClusters(clusters);
        for (auto &cluster : clusters) {
            cluster.m_id = m_id_counter++;
            cluster.mean();
        }


        if (m_old_clusters.size() == 0) {
            m_old_clusters.insert(m_old_clusters.begin(), clusters.begin(), clusters.end());
        } else {
            cout << "Searching:" << endl;
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


        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();


        double millis = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000.0;
        millis += std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1000000.0;

        std::cout << "Time difference = " << millis << std::endl;
        //cout << "--------------------------" << endl << endl;

        const static int res = 800;
        const static int zoom = 8;




        cv::Mat image(res, res, CV_8UC3, cv::Scalar(0, 0, 0));
        for (int i = 0; i < m_cloudSize; i++) {
            for (int j = 0; j < 16; j++) {
                Point *point = &m_points[i][j];
                int x = static_cast<int>(point->getX() * zoom) + res / 2;
                int y = static_cast<int>(point->getY() * zoom) + res / 2;
                if ((x < 800) && (y < 800) && (y >= 0) && (x >= 0)) {
                    // if (point->getMeasurement() > 1 && ( point->getPos()[3] <0  && point->getPos()[3] > -1)) {
                    image.at<cv::Vec3b>(y, x)[0] = 255;
                    image.at<cv::Vec3b>(y, x)[1] = 255;
                    image.at<cv::Vec3b>(y, x)[2] = 255;
                    //  }

                }

            }
        }


        for (auto &cluster : clusters) {
            for (auto point : (cluster.m_cluster)) {
                int x = static_cast<int>(point->getX() * zoom) + res / 2;
                int y = static_cast<int>(point->getY() * zoom) + res / 2;
                if ((x < res) && (y < res) && (y >= 0) && (x >= 0)) {
                    if (cluster.m_id % 3 == 0) {
                        image.at<cv::Vec3b>(y, x)[0] = 255;
                        image.at<cv::Vec3b>(y, x)[1] = 0;
                        image.at<cv::Vec3b>(y, x)[2] = 0;
                    }
                    if (cluster.m_id % 3 == 1) {
                        image.at<cv::Vec3b>(y, x)[0] = 0;
                        image.at<cv::Vec3b>(y, x)[1] = 255;
                        image.at<cv::Vec3b>(y, x)[2] = 0;
                    }
                    if (cluster.m_id % 3 == 2) {
                        image.at<cv::Vec3b>(y, x)[0] = 0;
                        image.at<cv::Vec3b>(y, x)[1] = 0;
                        image.at<cv::Vec3b>(y, x)[2] = 255;
                    }

                }
            }
        }

        for (auto &cluster : clusters) {
            auto hull = utils::convex_hull(cluster);
            std::stringstream ss;
            ss << cluster.m_id;

            cv::putText(image, ss.str(), cv::Point(cluster.m_center[0] * zoom + res / 2, cluster.m_center[1] * zoom + res / 2), cv::FONT_HERSHEY_SIMPLEX, 0.33,
                        cv::Scalar(255, 255, 255));
            for (int i = 1; i < hull.size(); i++) {
                cv::line(image, cv::Point(hull[i - 1]->getX() * zoom + res / 2, hull[i - 1]->getY() * zoom + res / 2),
                         cv::Point(hull[i]->getX() * zoom + res / 2, hull[i]->getY() * zoom + res / 2),
                         cv::Scalar(255, 255, 0), 1, 8, 0);
            }
            if (hull.size() > 1)
                cv::line(image, cv::Point(hull[hull.size() - 1]->getX() * zoom + res / 2, hull[hull.size() - 1]->getY() * zoom + res / 2),
                         cv::Point(hull[0]->getX() * zoom + res / 2, hull[0]->getY() * zoom + res / 2), cv::Scalar(255, 255, 0), 1, 8, 0);
        }


        cv::line(image, cv::Point(res-20 -zoom ,res-20), cv::Point(res-20,res-20), cv::Scalar(255, 255, 0), 1);


        cv::imshow("Display Image", image);

        cv::waitKey(1);
    }

}

