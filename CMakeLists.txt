cmake_minimum_required(VERSION 3.0.2)
project(voxel_svio)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  message_filters
  roscpp
  rospy
  sensor_msgs
  image_transport
  std_msgs
  tf
  cv_bridge
)

include(FetchContent)

FetchContent_Declare(
        tessil 
	SOURCE_DIR ${PROJECT_SOURCE_DIR}/thirdLibrary/tessil-src)

if (NOT tessil_POPULATED)
    set(BUILD_TESTING OFF)
    FetchContent_Populate(tessil)

    add_library(robin_map INTERFACE)
    add_library(tsl::robin_map ALIAS robin_map)

    target_include_directories(robin_map INTERFACE
            "$<BUILD_INTERFACE:${tessil_SOURCE_DIR}/include>")

    list(APPEND headers "${tessil_SOURCE_DIR}/include/tsl/robin_growth_policy.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_hash.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_map.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_set.h")
    target_sources(robin_map INTERFACE "$<BUILD_INTERFACE:${headers}>")

    if (MSVC)
        target_sources(robin_map INTERFACE
                "$<BUILD_INTERFACE:${tessil_SOURCE_DIR}/tsl-robin-map.natvis>")
    endif ()
endif ()

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  robin_map
  include
)

add_executable(
  vio_node
  src/camera.cpp
  src/frame.cpp
  src/pixelSelector.cpp
  src/feature.cpp
  src/featureHelper.cpp
  src/featureTracker.cpp
  src/initializer.cpp
  src/initializerHelper.cpp
  src/mapPoint.cpp
  src/mapManagement.cpp
  src/msckf.cpp
  src/parameters.cpp
  src/preIntegration.cpp
  src/quatOps.cpp
  src/sensorData.cpp
  src/state.cpp
  src/stateHelper.cpp
  src/stereoVio.cpp
  src/updaterHelper.cpp
  src/utility.cpp
)
target_link_libraries(vio_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} robin_map)
