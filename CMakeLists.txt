cmake_minimum_required(VERSION 3.4)
project(pointcloud_cluster)

set(CMAKE_CXX_STANDARD 11)

## set search path for od
SET(CMAKE_MODULE_PATH "${OPENDAVINCI_DIR}/share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules" ${CMAKE_MODULE_PATH})

FIND_PACKAGE(OpenDaVINCI REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(SYSTEM ${OPENDAVINCI_INCLUDE_DIRS})


# add scanned files as libs
add_library(${PROJECT_NAME}-utils STATIC src/Utils.cpp)
add_library(${PROJECT_NAME}-pointcloud-clustering STATIC src/dbscan.cpp src/PointcloudClustering.cpp src/Point.cpp)


# add od and scnanned libs to LIBRARIES
set(LIBRARIES ${OPENDAVINCI_LIBRARIES} ${PROJECT_NAME}-pointcloud-clustering ${PROJECT_NAME}-utils)

# set cflags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") ## -Wall -Wextra")
SET(CMAKE_CXX_FLAGS_RELEASE "-march=core-avx2 -O2 -pipe")

add_executable(${PROJECT_NAME} main.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LIBRARIES} ${OpenCV_LIBS})