#include <iostream>
#include "PointcloudClustering.h"

int32_t main(int32_t argc, char **argv) {
    PointcloudClustering dtr(argc, argv);

    return dtr.runModule();
}
