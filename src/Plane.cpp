#include "Plane.h"

double Plane::fitPlaneFromPoints(std::vector<Point *> &points) {
    // http://www.ilikebigbits.com/blog/2015/3/2/plane-from-points
    assert(points.size() > 3);
    Eigen::Vector3f sum(0, 0, 0);
    for (auto &point : points) {
        sum[0] = point->getX() + sum[0];
        sum[1] = point->getY() + sum[1];
        sum[2] = point->getZ() + sum[2];
    }
    Eigen::Vector3f center = sum / static_cast<float>(points.size());
    float xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;

    for (auto &point : points) {
        auto r = point->getVec() - center;
        xx += r[0] * r[0];
        xy += r[0] * r[1];
        xz += r[0] * r[2];
        yy += r[1] * r[1];
        yz += r[1] * r[2];
        zz += r[2] * r[2];
    }
    float det_x = yy * zz - yz * yz;
    float det_y = xx * zz - xz * xz;
    float det_z = xx * yy - xy * xy;



    // Pick path with best conditioning:
    Eigen::Vector3f dir;
    if (det_x >= det_y && det_x >= det_z)
        dir << 1.0, (xz * yz - xy * zz) / det_x, (xy * yz - xz * yy) / det_x;
    else if (det_y >= det_x && det_y >= det_z)
        dir << (yz * xz - xy * zz) / det_y, 1.0, (xy * xz - yz * xx) / det_y;
    else
        dir << (yz * xy - xz * yy) / det_z, (xz * xy - yz * xx) / det_z, 1.0;

    //plane_from_point_and_normal(&centroid, &normalize(dir))
    double sign = center.dot(dir);
    if (sign >= 0) {
        normal = dir / dir.norm();
    } else {
        normal = -dir / dir.norm();
    }

    distance = center.dot(normal);

    // calc error
    double error = 0;

    for (auto &point : points) {
        auto p = point->getVec();
        double tmp = p.dot(normal) - distance;
        error += tmp * tmp;


    }

    return error;
}



Plane::Plane(std::vector<Point *> &points){
    auto a=points[0]->getVec(),b=points[1]->getVec(),c=points[2]->getVec();
    auto dir = (b-a).cross(c-a);
    double sign = a.dot(dir);
    if (sign >= 0) {
        normal = dir / dir.norm();
    } else {
        normal = -dir / dir.norm();
    }
    distance = a.dot(normal);
}

float Plane::getDist(Eigen::Vector3f point){
    return point.dot(normal)-distance;

}