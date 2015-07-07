cmake_minimum_required(VERSION 2.8.3)
project(narf_place_recognition)

# Generate database for vim YouCompleteMe plugin
set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")

## Find catkin and any catkin packages
find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    tf
    tf_conversions
    pcl_ros
    pointmatcher_ros
    rosbag
    )

find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS filesystem system)
find_package(libpointmatcher 1.1.0 REQUIRED)
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    tf
    tf_conversions
    eigen
    pcl_ros
    pointmatcher_ros
    rosbag
    )

include_directories(include 
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${libpointmatcher_INCLUDE_DIRS}
    )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
add_executable(bag_to_dataset 
    src/bag_to_dataset.cpp
    src/DatasetGenerator.cpp
    src/IcpOdometry.cpp)
target_link_libraries(bag_to_dataset 
    ${catkin_LIBRARIES}
    ${libpointmatcher_LIBRARIES}
    ${Boost_LIBRARIES}
    )
