#include "pointcloud.h"
#include "Utils.h"

void Pointcloud::addPoint(Point &point) {
    int x = point.getX() + 100;
    int y = point.getY() + 100;
    if (x < 0)
        x = 0;
    else if (x > 199)
        x = 199;

    if (y < 0)
        y = 0;
    else if (y > 199)
        y = 199;

    m_points[x][y].push_back(point);
}


std::list<Point *> Pointcloud::getPointsNextTo(float x, float y, float dist) {
    std::list<Point *> points;
    int x_id = x + 100;
    int y_id = y + 100;
    int delta_index = std::ceil(dist);
    int x_low = utils::max((x_id - delta_index), 0);
    int x_upp = utils::min((x_id + delta_index), 199);
    int y_low = utils::max((y_id - delta_index), 0);
    int y_upp = utils::min((y_id + delta_index), 199);
    for (int ix = x_low; ix <= x_upp; ix++) {
        for (int iy = y_low; iy <= y_upp; iy++) {
            for (auto &point : m_points[ix][iy]) {
                if (point.get2Distance(x, y) < dist)
                    points.push_back(&point);

            }

        }
    }

    return points;


}

Pointcloud::Pointcloud() {}


void Pointcloud::clean() {
    for (int ix = 0; ix < 200; ix++) {
        for (int iy = 0; iy < 200; iy++)
            m_points[ix][iy].clear();
    }
}
