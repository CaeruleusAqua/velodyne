#include "PointcloudClustering.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <list>
#include <chrono>


using namespace std;

// We add some of OpenDaVINCI's namespaces for the sake of readability.
using namespace odcore::base::module;
using namespace odcore::data;

PointcloudClustering::PointcloudClustering(const int32_t &argc, char **argv) :
        DataTriggeredConferenceClientModule(argc, argv, "PointcloudClustering") {}

PointcloudClustering::~PointcloudClustering() {}

void PointcloudClustering::setUp() {
    cout << "This method is called before the component's body is executed." << endl;
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
}

void PointcloudClustering::tearDown() {
    cout << "This method is called after the program flow returns from the component's body." << endl;
}


std::vector<std::vector<Point>> PointcloudClustering::transform(CompactPointCloud cpc) {
    static const int maping[] = {-15, -13, -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11, 13, 15};
    std::string distances = cpc.getDistances();
    vector<double> azimuth_range = utils::linspace(utils::deg2rad(cpc.getStartAzimuth()), utils::deg2rad(cpc.getEndAzimuth()), distances.size() / 2 / 16);
    const half *data = reinterpret_cast<const half *>(distances.c_str());
    std::vector<std::vector<Point>> points(16);
    for (uint32_t i = 0; i < distances.size() / 2; i += 16) {
        for (uint32_t offset = 0; offset < 16; offset++) {
            float measurement = static_cast<float>(data[i + offset]);
            float azimuth = static_cast<float>(azimuth_range[i / 16]);
            float xy_range = measurement * cos(static_cast<float>(utils::deg2rad(maping[offset])));
            float x = xy_range * sin(azimuth);
            float y = xy_range * cos(azimuth);
            float z = measurement * sin(static_cast<float>(utils::deg2rad(maping[offset])));
            Point point(x, y, z, measurement, azimuth);
            point.setIndex(i / 16, offset);
            if (measurement <= 1 || !(z < 10 && z > -2)) {
                point.setVisited(true);
                point.clustered = true;
            }
            points[offset].push_back(point);
        }
    }
    return points;
}

std::vector<Point *> PointcloudClustering::regionQuery(Point *point, double eps) {
    //cout<<"Start---------------------------: "<<endl;
    int didx = 3;
    std::vector<Point *> collection;
    float distance = point->getMeasurement();
    int i = point->getI();
    for (int k = max(0, i - didx); k < min(i + didx, (int) m_points[0].size()); k++) {
        for (int l = 0; l < 16; l++) {
            float z = point->getPos()[2];
            if (0 > z && z > -2) {
                //cout<<"Dist: "<<distance<<"   Dist Comp: "<<m_points[j][i].get2Distance(m_points[l][k])  <<  endl;
                if (point->get2Distance(m_points[l][k]) < eps) {
                    collection.push_back(&m_points[l][k]);
                }
            }
        }

    }
    //cout<<"END---------------------------: "<<endl;
    return collection;
}


//std::vector<Point *> PointcloudClustering::regionQuery(Point *point, double eps) {
//    //cout<<"Start---------------------------: "<<endl;
//    std::vector<Point *> collection;
//    for (int k = 0; k < m_points[0].size(); k++) {
//        for (int l = 0; l < 16; l++) {
//            if (point->get2Distance(m_points[l][k]) < eps)
//                collection.push_back(&m_points[l][k]);
//        }
//    }
//    return collection;
//}


void PointcloudClustering::expandCluster(std::vector<Point *> &neighbors, std::vector<Point *> &cluster) {
    while (!neighbors.empty()) {
        Point *point = neighbors.back();
        neighbors.pop_back();

        //cout<<"neighbood Size: " << (neighbors.size()) <<  "  Visited: "<< point->isVisited()<<endl;
        if (!point->isVisited()) {
            point->setVisited(true);
            std::vector<Point *> collection = regionQuery(point, 0.50);
            // if(collection.size() > 0)
            //     cout << "neighbors_ext : " << collection.size() << endl;
            if (collection.size() > 5) {
                neighbors.insert(neighbors.end(), collection.begin(), collection.end());
            }
        }
        if (!point->clustered) {
            cluster.push_back(point);
            point->clustered = true;
        }
    }
}


void PointcloudClustering::nextContainer(Container &c) {

    if (c.getDataType() == CompactPointCloud::ID()) {


        CompactPointCloud cpc = c.getData<CompactPointCloud>();

        m_points = transform(cpc);

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        std::vector<std::vector<Point *>> clusters;
        uint32_t c_num = 0;

        for (int i = 0; i < m_points[0].size(); i++) {
            for (int j = 0; j < 16; j++) {
                Point *point = &m_points[j][i];
                if (!point->isVisited()) {
                    point->setVisited(true);
                    std::vector<Point *> collection = regionQuery(point, 0.50);
                    //if(collection.size() > 0)
                    //    cout << "neighbors : " << collection.size() << endl;
                    if (collection.size() > 5) {
                        std::vector<Point *> cluster;
                        cluster.push_back(point);
                        point->clustered = true;

                        expandCluster(collection, cluster);
                        clusters.push_back(cluster);
                    }

                }
            }
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();


        double millis = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000.0;
        millis += std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1000000.0;

        std::cout << "Time difference = " << millis << std::endl;
        //cout << "Clusters Num: " << clusters.size() << endl;
        //for (uint32_t i = 0; i < clusters.size(); i++) {
        //    cout << clusters[i].size() << endl;
        //}
        //cout << "--------------------------" << endl << endl;

        cv::Mat image(800, 800, CV_8UC3, cv::Scalar(0, 0, 0));
        unsigned char *input = (unsigned char *) (image.data);
        for (int i = 0; i < m_points[0].size(); i++) {
            for (int j = 0; j < 16; j++) {
                Point *point = &m_points[j][i];
                int x = static_cast<int>(point->getPos()[0] * 8) + 400;
                int y = static_cast<int>(point->getPos()[1] * 8) + 400;
                if ((x < 800) && (y < 800) && (y >= 0) && (x >= 0)) {
                    // if (point->getMeasurement() > 1 && ( point->getPos()[3] <0  && point->getPos()[3] > -1)) {
                    image.at<cv::Vec3b>(y, x)[0] = 255;
                    image.at<cv::Vec3b>(y, x)[1] = 255;
                    image.at<cv::Vec3b>(y, x)[2] = 255;
                    //  }

                }

            }
        }

        int i = 0;
        for (auto cluster : clusters) {
            for (auto point : cluster) {
                int x = static_cast<int>(point->getPos()[0] * 8) + 400;
                int y = static_cast<int>(point->getPos()[1] * 8) + 400;
                if ((x < 800) && (y < 800) && (y >= 0) && (x >= 0)) {
                    if (i % 3 == 0) {
                        image.at<cv::Vec3b>(y, x)[0] = 255;
                        image.at<cv::Vec3b>(y, x)[1] = 0;
                        image.at<cv::Vec3b>(y, x)[2] = 0;
                    }
                    if (i % 3 == 1) {
                        image.at<cv::Vec3b>(y, x)[0] = 0;
                        image.at<cv::Vec3b>(y, x)[1] = 255;
                        image.at<cv::Vec3b>(y, x)[2] = 0;
                    }
                    if (i % 3 == 2) {
                        image.at<cv::Vec3b>(y, x)[0] = 0;
                        image.at<cv::Vec3b>(y, x)[1] = 0;
                        image.at<cv::Vec3b>(y, x)[2] = 255;
                    }

                }
            }
            i++;
        }


        cv::imshow("Display Image", image);

        cv::waitKey(1);
    }

}

