#include "Utils.h"
#include "Cluster.h"
#include "Point.h"

namespace utils {


    std::vector<double> range(double start, double stop, double step, bool endpoint) {
        if (start > stop && step > 0) {
            step = -step;
        }
        std::vector<double> array;
        uint64_t num = 0;
        if (endpoint) {
            num = std::abs((stop - start) / step) + 1;
        } else {
            num = std::abs((stop - start - step) / step) + 1;
        }
        std::cout << "Num: " << num << std::endl;

        array.reserve(num);
        if (step >= 0) {
            for (uint64_t i = 0; i < num; i++) {
                array.push_back(start);
                start += step;
            }
        } else {
            for (uint64_t i = 0; i < num; i++) {
                array.push_back(start);
                start += step;
            }
        }
        return array;
    }

    uint32_t getMaxIndex(std::vector<Point *> array) {
        float maxz = array[0]->getZ();
        uint32_t max_index = 0;
        for (uint32_t i = 1; i < array.size(); i++) {
            if (array[i]->getZ() > maxz) {
                maxz = array[i]->getZ();
                max_index = i;
            }
        }
        return max_index;
    }

    std::vector<Point *> minZinSec(int start, int stop, Point (&points)[2000][16]) {
        static const uint32_t num_of_minima = 14;
        std::vector<Point *> array;

        for (uint32_t i = start; i < start+num_of_minima; i++) {
            array.push_back(&(points[0][14]));    // this could be wrong...
        }


        uint32_t endOffset = 3; // assume only negative angles for minimal z values
        for (int i = start; i < stop; i++) {
            for (uint32_t offset = 0; offset < endOffset; offset++) {
                int maxIdx = getMaxIndex(array);
                if (points[i][offset].getZ() < array[maxIdx]->getZ()) {
                    array[maxIdx] = &(points[i][offset]);
                }
            }
        }

        return array;
    }

    std::vector<double> linspace(double start, double stop, uint64_t num, bool endpoint) {
        std::vector<double> array;
        double step = 0;
        if (endpoint) {
            if (num == 1) {
                array.push_back(start);
                return array;
            }
            step = (stop - start) / (num - 1);
        } else {
            if (num == 0) {
                return array;
            }
            step = (stop - start) / (num);
        }
        array.reserve(num);
        if (start == stop) {
            for (uint64_t i = 0; i < num; i++) {
                array.push_back(start);
            }
        } else if (step >= 0) {
            for (uint64_t i = 0; i < num; i++) {
                array.push_back(start);
                start += step;
            }
        } else {
            for (uint64_t i = 0; i < num; i++) {
                array.push_back(start);
                start += step;
            }
        }
        return array;
    }

    // 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
    // Returns a positive value, if OAB makes a counter-clockwise turn,
    // negative for clockwise turn, and zero if the points are collinear.
    double cross(const Point *O, const Point *A, const Point *B) {
        return (A->getX() - O->getX()) * (B->getY() - O->getY()) - (A->getY() - O->getY()) * (B->getX() - O->getX());
    }

    // Returns a list of points on the convex hull in counter-clockwise order.
    // Note: the last point in the returned list is the same as the first one.
    std::vector<Point *> convex_hull(Cluster P) {
        int n = P.m_cluster.size(), k = 0;
        std::vector<Point *> H(2 * n);

        // Sort points lexicographically
        std::sort(P.m_cluster.begin(), P.m_cluster.end(), less_than_key());

        // Build lower hull
        for (int i = 0; i < n; ++i) {
            while (k >= 2 && cross(H[k - 2], H[k - 1], P.m_cluster[i]) <= 0) k--;
            H[k++] = P.m_cluster[i];
        }

        // Build upper hull
        for (int i = n - 2, t = k + 1; i >= 0; i--) {
            while (k >= t && cross(H[k - 2], H[k - 1], P.m_cluster[i]) <= 0) k--;
            H[k++] = P.m_cluster[i];
        }

        H.resize(k - 1);
        return H;
    }

}


